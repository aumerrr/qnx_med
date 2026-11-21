#!/bin/sh
# show_log.sh - runs ON THE PI. Re-prints the last lines of the control-loop log every second
# (QNX tail -f is not available here). Ctrl-C to stop.   usage: sh /tmp/show_log.sh [logfile]
LOG=${1:-/tmp/ctl_loop.log}
while true; do
    clear
    echo "$LOG   (Ctrl-C to stop)"
    echo "-------------------------------------------------------------"
    tail -n 20 "$LOG" 2>/dev/null || echo "(no log yet - waiting for ctl_loop)"
    sleep 1
done
