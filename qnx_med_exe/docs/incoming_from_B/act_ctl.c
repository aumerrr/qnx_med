/* act_ctl.c - test tool for /dev/actuator devctl (Person B)
 * usage: act_ctl safe | run | stats
 */
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <devctl.h>

#include "actuator_ipc.h"

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s safe | run | stats\n", argv[0]);
        return 2;
    }

    int fd = open(ACT_DEV_PATH, O_RDWR);
    if (fd < 0) { perror("open " ACT_DEV_PATH); return 1; }

    int r = 0;
    if (strcmp(argv[1], "safe") == 0) {
        r = devctl(fd, DCMD_ACT_SAFE, NULL, 0, NULL);
        if (r == 0) printf("SAFE set\n");
    } else if (strcmp(argv[1], "run") == 0) {
        r = devctl(fd, DCMD_ACT_RUN, NULL, 0, NULL);
        if (r == 0) printf("RUN set\n");
    } else if (strcmp(argv[1], "stats") == 0) {
        act_stats_t s;
        r = devctl(fd, DCMD_ACT_STATS, &s, sizeof(s), NULL);
        if (r == 0)
            printf("state=%s last_us=%u writes=%u rejects=%u ignored=%u safe_count=%u\n",
                   s.state == ACT_STATE_SAFE ? "SAFE" : "RUN",
                   s.last_us, s.writes, s.rejects, s.ignored, s.safe_count);
    } else {
        fprintf(stderr, "unknown command: %s\n", argv[1]);
        close(fd);
        return 2;
    }

    if (r != 0) fprintf(stderr, "devctl failed: %s\n", strerror(r));
    close(fd);
    return r ? 1 : 0;
}
