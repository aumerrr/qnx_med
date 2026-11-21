#!/bin/sh
# run_all.sh - runs ON THE PI as root (started by scripts/demo.ps1, or by hand: sh /tmp/run_all.sh).
# Stops old copies, starts actuator_rm -> pot source -> ctl_loop -> guardian in the background,
# then opens the dashboard. When you press q in the dashboard, everything is stopped.
#   guardian = recovery manager: puts the actuator in SAFE if the loop is starved and, after
#   GRACE_MS (default 1000), lowers the priority of the load threads that cause it.
#   Override:  GRACE_MS=3000 LOAD_SECS=30 sh /tmp/run_all.sh
#   usage: sh /tmp/run_all.sh [csv_name]        (CSV goes to /tmp/<csv_name>)
#   logs:  /tmp/actuator_rm.log  /tmp/adc_reader.log  /tmp/ctl_loop.log  /tmp/guardian.log
#
# Two modes, chosen automatically:
#   CLUSTER mode - the Pi booted with a cluster whose cores are CPU1+CPU2 (mask 0x6):
#       the three critical programs start inside that cluster ("on -c <name>", or "on -R 0x6" if this
#       Pi's "on" has no -c), the guardian and the dashboard run on core 0 ("on -C0"),
#       core 3 is left for the contained test load, and the programs do NOT pin themselves
#       (RT_NO_PIN=1).
#       Override the cluster name with:  CLUSTER=<name> sh /tmp/run_all.sh
#   FLAT mode    - no such cluster: exactly the old behaviour (programs pin themselves).
# Only the 7 project programs are started (actuator_rm adc_reader ctl_loop guardian monitor and the
# loads cpu_hog msg_flood). The test tools (fake_pot, fake_stats, act_ctl, pot_servo, pot_mirror)
# are NOT started and not copied; if /dev/i2c1 is missing the script stops with an error.
# Everything is stopped automatically: on q in the dashboard, on Ctrl-C, and if the SSH window
# is closed or the connection drops (trap below).
# Keep this file with LF line endings (QNX sh fails on Windows CRLF).

cd /tmp || exit 1
I2C=${I2C:-/dev/i2c1}
RAW_MIN=${RAW_MIN:-0}
RAW_MAX=${RAW_MAX:-26400}
CSV=${1:-run.csv}
GRACE_MS=${GRACE_MS:-1000}
LOAD_SECS=${LOAD_SECS:-20}

# Loads first with SIGKILL (starved loads ignore a normal stop), then our own programs normally
# so actuator_rm can leave the servo pin low. The old test-tool names are included so that stale
# copies from earlier runs are cleaned up too.
stop_all() {
    slay -f -s KILL cpu_hog msg_flood >/dev/null 2>&1
    slay -f guardian monitor ctl_loop adc_reader actuator_rm fake_pot fake_stats pot_servo pot_mirror >/dev/null 2>&1
}
STOPPED=0
cleanup() {
    if [ "$STOPPED" = 0 ]; then
        STOPPED=1
        trap '' 1 2 3 15
        echo "stopping everything"
        stop_all
        sleep 1
        slay -f -s KILL cpu_hog msg_flood guardian monitor ctl_loop adc_reader actuator_rm >/dev/null 2>&1
        chmod a+r "/tmp/$CSV" 2>/dev/null
    fi
}
# 1=HUP (SSH closed / dropped)  2=Ctrl-C  3=QUIT  15=TERM
trap 'cleanup; exit 1' 1 2 3 15

# ---- find the cluster that covers CPU1+CPU2 (mask 0x6) ----
if [ -z "$CLUSTER" ]; then
    CLUSTER=$(pidin syspage=cluster 2>/dev/null | grep "cpumask:0*6 " | sed -n 's/.*name:[0-9]*(\([^)]*\)).*/\1/p' | head -n 1)
fi
# Some SDK builds of "on" do not know -c (error: invalid option -- c). Use -c when it works,
# otherwise "-R 0x6" (same result: runmask AND inherit mask = 0x6, valid because 0x6 is a cluster).
ON_RT_OK=""
if [ -n "$CLUSTER" ] && pidin syspage=cluster 2>/dev/null | grep -q "($CLUSTER)"; then
    if on -c "$CLUSTER" true >/dev/null 2>&1; then
        ON_RT_OK="on -c $CLUSTER"
    elif on -R 0x6 true >/dev/null 2>&1; then
        ON_RT_OK="on -R 0x6"
    fi
fi
if [ -n "$ON_RT_OK" ] && on -C0 true >/dev/null 2>&1; then
    MODE=cluster
    ON_RT="$ON_RT_OK"
    ON_MON="on -C0"
    if on -R 0x6 true >/dev/null 2>&1; then RTMASK=0x6; else RTMASK=0; fi
    export RT_NO_PIN=1
    echo "MODE: CLUSTER - critical programs started with '$ON_RT' (cluster '$CLUSTER', cores 1-2), guardian + dashboard on core 0"
else
    MODE=flat
    ON_RT=""
    ON_MON=""
    RTMASK=0
    unset RT_NO_PIN
    echo "MODE: FLAT - no cluster found, programs pin themselves (old behaviour)"
fi

echo "[1/5] stopping old copies"
stop_all
sleep 1
chmod +x actuator_rm adc_reader ctl_loop guardian monitor cpu_hog msg_flood 2>/dev/null

if [ ! -e "$I2C" ]; then
    echo "ERROR: $I2C not found - start the I2C driver first (the potentiometer is read through it)"; exit 1
fi

echo "[2/5] actuator_rm"
$ON_RT ./actuator_rm > /tmp/actuator_rm.log 2>&1 &
if ! waitfor /dev/actuator 5 >/dev/null 2>&1; then
    echo "ERROR: /dev/actuator did not appear"; cat /tmp/actuator_rm.log; stop_all; exit 1
fi

echo "[3/5] adc_reader $I2C $RAW_MIN $RAW_MAX"
$ON_RT ./adc_reader "$I2C" "$RAW_MIN" "$RAW_MAX" > /tmp/adc_reader.log 2>&1 &
if ! waitfor /dev/shmem/pot 5 >/dev/null 2>&1; then
    echo "ERROR: /pot did not appear"; cat /tmp/adc_reader.log; stop_all; exit 1
fi
sleep 1

echo "[4/5] ctl_loop"
$ON_RT ./ctl_loop -v > /tmp/ctl_loop.log 2>&1 &
sleep 2
head -4 /tmp/ctl_loop.log

echo "[4b/5] guardian (recovery manager, grace ${GRACE_MS} ms)"
if [ -x ./guardian ]; then
    $ON_MON ./guardian -g "$GRACE_MS" > /tmp/guardian.log 2>&1 &
    sleep 1
    head -2 /tmp/guardian.log
    if ! pidin -p guardian >/dev/null 2>&1 || [ ! -e /dev/shmem/guard_stats ]; then
        echo "!!!! guardian FAILED to start - log follows. Loads on the RT cores will NOT be recovered."
        cat /tmp/guardian.log
        sleep 3
    fi
else
    echo "!!!! guardian not found in /tmp (was it built? is it in bin/?) - no automatic recovery from load"
    sleep 3
fi

if grep -q -i "warning\|error\|cannot" /tmp/ctl_loop.log /tmp/adc_reader.log /tmp/actuator_rm.log /tmp/guardian.log 2>/dev/null; then
    echo "!! warnings above - check them (are you root?)"
fi

# ---- show where every thread is allowed to run (Runmask 0x6 = cores 1-2, 0x8 = core 3) ----
echo "---- placement check (count of threads per Runmask) ----"
for p in actuator_rm adc_reader ctl_loop guardian; do
    echo "$p:"
    pidin -p $p rmask 2>/dev/null | grep "Runmask" | sort | uniq -c
done

echo "[5/5] dashboard - keys: l CPU load, m message flood, b contained load, k stop, r reset, c CSV, q quit"
sleep 2
$ON_MON ./monitor -m "$RTMASK" -T "$LOAD_SECS" -l "/tmp/$CSV"

echo "dashboard closed"
cleanup
echo "CSV: /tmp/$CSV"
