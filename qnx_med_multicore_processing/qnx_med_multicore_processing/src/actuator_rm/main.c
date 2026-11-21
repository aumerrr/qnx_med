/* actuator_rm.c - /dev/actuator resource manager (Person B)
 *
 * write : ASCII pulse width in us  -> validated, then servo_set_us()
 * read  : "<last_us> RUN|SAFE\n"
 * devctl: SAFE / RUN / STATS  (see actuator_ipc.h)
 *
 * While SAFE, valid writes are accepted but ignored (counted), so the control
 * loop does not need to special-case the safe state. DCMD_ACT_RUN resumes.
 *
 * Only this process owns the servo (servo.c, GPIO22).
 */
/* thread pool callbacks take a dispatch_context_t (must precede the QNX includes) */
#define THREAD_POOL_PARAM_T dispatch_context_t

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <devctl.h>

#include "servo.h"
#include "actuator_ipc.h"

/* Server threads run on the client's cores (run mask inherited from the
 * sender). If the build says ChannelCreate / dispatch_create_channel are
 * undefined, delete this line to fall back to dispatch_create(). */
#define ACT_INHERIT_RUNMASK 1

static pthread_mutex_t g_lock;
static act_stats_t     g_st;

static resmgr_connect_funcs_t connect_funcs;
static resmgr_io_funcs_t      io_funcs;
static iofunc_attr_t          attr;

/* ---------- state helpers (all under g_lock) ---------- */

static void act_enter_safe(void)
{
    pthread_mutex_lock(&g_lock);
    servo_set_us(ACT_SAFE_PULSE_US);
    g_st.last_us = ACT_SAFE_PULSE_US;
    if (g_st.state != ACT_STATE_SAFE) {
        g_st.state = ACT_STATE_SAFE;
        g_st.safe_count++;
    }
    pthread_mutex_unlock(&g_lock);
}

static void act_enter_run(void)
{
    pthread_mutex_lock(&g_lock);
    g_st.state = ACT_STATE_RUN;
    pthread_mutex_unlock(&g_lock);
}

/* ---------- write ---------- */

static int io_write(resmgr_context_t *ctp, io_write_t *msg, iofunc_ocb_t *ocb)
{
    int status = iofunc_write_verify(ctp, msg, ocb, NULL);
    if (status != EOK) return status;

    if ((msg->i.xtype & _IO_XTYPE_MASK) != _IO_XTYPE_NONE) return ENOSYS;

    size_t nbytes = msg->i.nbytes;
    char buf[16];

    if (nbytes == 0 || nbytes > sizeof(buf) - 1) {
        pthread_mutex_lock(&g_lock); g_st.rejects++; pthread_mutex_unlock(&g_lock);
        return EINVAL;
    }
    if (resmgr_msgread(ctp, buf, nbytes, sizeof(msg->i)) == -1) return errno;
    buf[nbytes] = '\0';

    char *end;
    long v = strtol(buf, &end, 10);
    int  ok = (end != buf) && (*end == '\0' || *end == '\n')
              && v >= SERVO_MIN_US && v <= SERVO_MAX_US;

    pthread_mutex_lock(&g_lock);
    if (!ok) {
        g_st.rejects++;
        pthread_mutex_unlock(&g_lock);
        return EINVAL;
    }
    if (g_st.state == ACT_STATE_SAFE) {
        g_st.ignored++;
    } else {
        servo_set_us((int)v);
        g_st.last_us = (uint32_t)v;
        g_st.writes++;
    }
    pthread_mutex_unlock(&g_lock);

    _IO_SET_WRITE_NBYTES(ctp, nbytes);
    return _RESMGR_NPARTS(0);
}

/* ---------- read ---------- */

static int io_read(resmgr_context_t *ctp, io_read_t *msg, iofunc_ocb_t *ocb)
{
    int status = iofunc_read_verify(ctp, msg, ocb, NULL);
    if (status != EOK) return status;

    if ((msg->i.xtype & _IO_XTYPE_MASK) != _IO_XTYPE_NONE) return ENOSYS;

    char buf[48];
    pthread_mutex_lock(&g_lock);
    int n = snprintf(buf, sizeof(buf), "%u %s\n", (unsigned)g_st.last_us,
                     g_st.state == ACT_STATE_SAFE ? "SAFE" : "RUN");
    pthread_mutex_unlock(&g_lock);

    int off   = (int)ocb->offset;
    int nleft = n - off;
    if (nleft < 0) nleft = 0;
    int nbytes = (int)msg->i.nbytes < nleft ? (int)msg->i.nbytes : nleft;

    if (nbytes > 0) {
        ocb->offset += nbytes;
        MsgReply(ctp->rcvid, nbytes, buf + off, nbytes);   /* copies now */
        return _RESMGR_NOREPLY;
    }
    _IO_SET_READ_NBYTES(ctp, 0);
    return _RESMGR_NPARTS(0);
}

/* ---------- devctl ---------- */

static int io_devctl(resmgr_context_t *ctp, io_devctl_t *msg, iofunc_ocb_t *ocb)
{
    int status = iofunc_devctl_default(ctp, msg, ocb);
    if (status != _RESMGR_DEFAULT) return status;

    int nbytes = 0;
    switch (msg->i.dcmd) {
    case DCMD_ACT_SAFE:
        act_enter_safe();
        break;
    case DCMD_ACT_RUN:
        act_enter_run();
        break;
    case DCMD_ACT_STATS: {
        act_stats_t *out = (act_stats_t *)_DEVCTL_DATA(msg->o);
        pthread_mutex_lock(&g_lock);
        *out = g_st;
        pthread_mutex_unlock(&g_lock);
        nbytes = sizeof(*out);
        break;
    }
    default:
        return ENOSYS;
    }

    memset(&msg->o, 0, sizeof(msg->o));
    msg->o.ret_val = EOK;
    msg->o.nbytes  = nbytes;
    return _RESMGR_PTR(ctp, &msg->o, sizeof(msg->o) + nbytes);
}

/* ---------- main ---------- */

int main(void)
{
    /* priority-inheriting mutex: a low-priority client holding g_lock cannot
     * block a high-priority control-loop write (priority inversion). */
    pthread_mutexattr_t ma;
    pthread_mutexattr_init(&ma);
    pthread_mutexattr_setprotocol(&ma, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&g_lock, &ma);

    if (servo_init() != 0) {            /* assumes 0 = OK; check servo.c */
        fprintf(stderr, "actuator_rm: servo_init failed\n");
        return 1;
    }
    servo_set_us(ACT_SAFE_PULSE_US);    /* start at centre */
    memset(&g_st, 0, sizeof(g_st));
    g_st.state   = ACT_STATE_RUN;
    g_st.last_us = ACT_SAFE_PULSE_US;

    dispatch_t *dpp;
#ifdef ACT_INHERIT_RUNMASK
    int chid = ChannelCreate(_NTO_CHF_INHERIT_RUNMASK);
    if (chid == -1) { perror("ChannelCreate"); return 1; }
    dpp = dispatch_create_channel(chid, 0);
#else
    dpp = dispatch_create();
#endif
    if (dpp == NULL) { perror("dispatch_create"); return 1; }

    resmgr_attr_t rattr;
    memset(&rattr, 0, sizeof(rattr));
    rattr.nparts_max   = 1;
    rattr.msg_max_size = 2048;

    iofunc_func_init(_RESMGR_CONNECT_NFUNCS, &connect_funcs,
                     _RESMGR_IO_NFUNCS, &io_funcs);
    io_funcs.read   = io_read;
    io_funcs.write  = io_write;
    io_funcs.devctl = io_devctl;

    iofunc_attr_init(&attr, S_IFCHR | 0666, NULL, NULL);

    if (resmgr_attach(dpp, &rattr, ACT_DEV_PATH, _FTYPE_ANY, 0,
                      &connect_funcs, &io_funcs, &attr) == -1) {
        perror("resmgr_attach");
        return 1;
    }

    thread_pool_attr_t tpa;
    memset(&tpa, 0, sizeof(tpa));
    tpa.handle        = dpp;
    tpa.context_alloc = dispatch_context_alloc;
    tpa.context_free  = dispatch_context_free;
    tpa.block_func    = dispatch_block;
    tpa.unblock_func  = dispatch_unblock;
    tpa.handler_func  = dispatch_handler;
    tpa.lo_water      = 2;
    tpa.hi_water      = 4;
    tpa.increment     = 1;
    tpa.maximum       = 8;

    thread_pool_t *pool = thread_pool_create(&tpa, POOL_FLAG_USE_SELF);
    if (pool == NULL) { perror("thread_pool_create"); return 1; }

    printf("actuator_rm: " ACT_DEV_PATH " ready\n");
    fflush(stdout);

    thread_pool_start(pool);            /* does not return */
    return 0;
}
