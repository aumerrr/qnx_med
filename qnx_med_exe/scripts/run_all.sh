#!/bin/sh
# run_all.sh - runs ON THE PI as root (started by scripts/demo.ps1, or by hand: sh /tmp/run_all.sh).
# Stops old copies, starts actuator_rm -> adc_reader -> ctl_loop in the background,
# then opens the dashboard. When you press q in the dashboard, everything is stopped.
#   usage: sh /tmp/run_all.sh [csv_name]        (CSV goes to /tmp/<csv_name>)
#   logs:  /tmp/actuator_rm.log  /tmp/adc_reader.log  /tmp/ctl_loop.log
# Keep this file with LF line endings (QNX sh fails on Windows CRLF).

cd /tmp || exit 1
I2C=${I2C:-/dev/i2c1}
RAW_MIN=${RAW_MIN:-0}
RAW_MAX=${RAW_MAX:-26400}
CSV=${1:-run.csv}

stop_all() {
    slay actuator_rm adc_reader ctl_loop monitor cpu_hog fake_stats fake_pot pot_servo pot_mirror >/dev/null 2>&1
}

echo "[1/5] stopping old copies"
stop_all
sleep 1
chmod +x actuator_rm adc_reader ctl_loop monitor cpu_hog act_ctl 2>/dev/null

if [ ! -e "$I2C" ]; then
    echo "ERROR: $I2C not found - start the I2C driver first"; exit 1
fi

echo "[2/5] actuator_rm"
./actuator_rm > /tmp/actuator_rm.log 2>&1 &
if ! waitfor /dev/actuator 5 >/dev/null 2>&1; then
    echo "ERROR: /dev/actuator did not appear"; cat /tmp/actuator_rm.log; stop_all; exit 1
fi

echo "[3/5] adc_reader $I2C $RAW_MIN $RAW_MAX"
./adc_reader "$I2C" "$RAW_MIN" "$RAW_MAX" > /tmp/adc_reader.log 2>&1 &
if ! waitfor /dev/shmem/pot 5 >/dev/null 2>&1; then
    echo "ERROR: /pot did not appear"; cat /tmp/adc_reader.log; stop_all; exit 1
fi
sleep 1

echo "[4/5] ctl_loop"
./ctl_loop -v > /tmp/ctl_loop.log 2>&1 &
sleep 2
head -4 /tmp/ctl_loop.log
if grep -q -i "warning\|error\|cannot" /tmp/ctl_loop.log /tmp/adc_reader.log /tmp/actuator_rm.log; then
    echo "!! warnings above - check them (are you root?)"
fi

echo "[5/5] dashboard - keys: l load on (cpu_hog), k load off, r reset, c CSV, q quit"
sleep 2
./monitor -l "/tmp/$CSV"

echo "dashboard closed - stopping everything"
stop_all
chmod a+r "/tmp/$CSV" 2>/dev/null
echo "CSV: /tmp/$CSV"
