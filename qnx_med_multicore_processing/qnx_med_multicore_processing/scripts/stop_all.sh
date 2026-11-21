#!/bin/sh
# stop_all.sh - runs ON THE PI as root. Stops every program of the project, loads first.
#   usage: sh /tmp/stop_all.sh
# Loads get SIGKILL (they may be too starved to react to a normal stop); our own programs get
# a normal stop so actuator_rm can leave the servo pin low, then anything left gets SIGKILL.
# Old test-tool names are included so stale copies from earlier runs are cleaned up too.
slay -f -s KILL cpu_hog msg_flood >/dev/null 2>&1
slay -f guardian monitor ctl_loop adc_reader actuator_rm fake_pot fake_stats pot_servo pot_mirror >/dev/null 2>&1
sleep 1
slay -f -s KILL guardian monitor ctl_loop adc_reader actuator_rm fake_pot fake_stats pot_servo pot_mirror >/dev/null 2>&1
echo "still running (should be empty):"
pidin -f "%a %N" | grep -E "cpu_hog|msg_flood|guardian|monitor|ctl_loop|adc_reader|fake_pot|actuator_rm" || echo "  nothing"
