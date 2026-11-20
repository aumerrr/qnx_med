/* actuator_ipc.h - interface of /dev/actuator (Person B)
 * Shared by actuator_rm (server), act_ctl (test tool) and the control loop.
 * Later this moves into common/ipc_defs.h.
 */
#ifndef ACTUATOR_IPC_H
#define ACTUATOR_IPC_H

#include <stdint.h>
#include <devctl.h>

#define ACT_DEV_PATH       "/dev/actuator"
#define ACT_SAFE_PULSE_US  1500          /* predefined safe response (centre) */

#define ACT_STATE_RUN      0
#define ACT_STATE_SAFE     1

/* returned by DCMD_ACT_STATS */
typedef struct {
    uint32_t state;        /* ACT_STATE_RUN / ACT_STATE_SAFE            */
    uint32_t last_us;      /* last pulse width applied to the servo     */
    uint32_t writes;       /* accepted writes applied to the servo      */
    uint32_t rejects;      /* writes refused (bad text / out of range)  */
    uint32_t ignored;      /* valid writes dropped while in SAFE        */
    uint32_t safe_count;   /* number of RUN -> SAFE transitions         */
} act_stats_t;

/* write: ASCII decimal pulse width in us, e.g. "1500"
 * read : "<last_us> RUN|SAFE\n"
 * devctl:
 *   DCMD_ACT_SAFE   no data. Apply the safe pulse now, latch SAFE.
 *   DCMD_ACT_RUN    no data. Leave SAFE, accept writes again.
 *   DCMD_ACT_STATS  returns act_stats_t.
 */
#define DCMD_ACT_SAFE   __DION(_DCMD_MISC, 0x41)
#define DCMD_ACT_RUN    __DION(_DCMD_MISC, 0x42)
#define DCMD_ACT_STATS  __DIOF(_DCMD_MISC, 0x43, act_stats_t)

#endif
