# demo.ps1 - run on the WINDOWS laptop. One command does everything:
#   copies bin\* and run_all.sh to the Pi, logs in, becomes root, starts the whole chain,
#   opens the dashboard, and after you press q copies the CSV back to results\.
#
#   powershell -ExecutionPolicy Bypass -File scripts\demo.ps1               full run
#   powershell -ExecutionPolicy Bypass -File scripts\demo.ps1 -NoCopy       don't re-copy the programs (same build)
#   powershell -ExecutionPolicy Bypass -File scripts\demo.ps1 -Logs         also open a window with ctl_loop's log
#   powershell -ExecutionPolicy Bypass -File scripts\demo.ps1 -Stop         stop everything on the Pi
#   (a normal run also stops everything by itself: on q, on Ctrl-C and if the connection drops)
#
# You will be asked for: the qnxuser password (copy), the qnxuser password (login), the root password (su).
param(
    [string]$Pi   = "10.0.0.1",
    [string]$User = "qnxuser",
    [string]$Csv  = ("run_" + (Get-Date -Format "yyyyMMdd_HHmm") + ".csv"),
    [switch]$NoCopy,
    [switch]$Logs,
    [switch]$Stop
)

$Root    = Split-Path -Parent $PSScriptRoot          # qnx_med_exe
$Bin     = Join-Path $Root "bin"
$Results = Join-Path $Root "results"
$Target  = "$User@$Pi"
$SshOpt  = @("-m", "hmac-sha2-256")
$ScpOpt  = @("-o", "MACs=hmac-sha2-256")

function Step($text) { Write-Host ""; Write-Host "==> $text" -ForegroundColor Cyan }

if ($Stop) {
    Step "Stopping everything on the Pi (root password)"
    ssh @SshOpt -t $Target "su root -c 'slay -f -s KILL cpu_hog msg_flood; slay -f guardian monitor ctl_loop adc_reader actuator_rm fake_pot; sleep 1; slay -f -s KILL guardian monitor ctl_loop adc_reader actuator_rm fake_pot'"
    exit
}

# The Pi scripts are always copied (small); the binaries only without -NoCopy.
# ONLY the programs the system needs are copied - the test tools in bin\ (fake_pot, fake_stats,
# act_ctl, pot_servo, pot_mirror, overrun_inject) stay on the laptop.
$needed = @("actuator_rm", "adc_reader", "ctl_loop", "guardian", "monitor", "cpu_hog", "msg_flood")
$files = @((Join-Path $PSScriptRoot "run_all.sh"), (Join-Path $PSScriptRoot "show_log.sh"), (Join-Path $PSScriptRoot "stop_all.sh"))
if (-not $NoCopy) {
    if (-not (Test-Path $Bin)) { Write-Host "No bin folder - build the project in Momentics first." -ForegroundColor Red; exit 1 }
    foreach ($n in $needed) {
        $f = Join-Path $Bin $n
        if (-not (Test-Path $f)) { Write-Host "Missing bin\$n - build the project in Momentics first." -ForegroundColor Red; exit 1 }
        $files += $f
    }
}
Step "Copying $($files.Count) files to ${Target}:/tmp (qnxuser password)"
scp @ScpOpt $files "${Target}:/tmp/"
if ($LASTEXITCODE -ne 0) { Write-Host "Copy failed." -ForegroundColor Red; exit 1 }

if ($Logs) {
    Step "Opening a second window with the ctl_loop log (qnxuser password there)"
    Start-Process powershell -ArgumentList "-NoExit", "-Command", "ssh -m hmac-sha2-256 -t $Target sh /tmp/show_log.sh"
}

Step "Starting the system on the Pi (qnxuser password, then root password)"
ssh @SshOpt -t $Target "su root -c 'sh /tmp/run_all.sh $Csv'"

Step "Copying the CSV back to $Results (qnxuser password)"
New-Item -ItemType Directory -Force $Results | Out-Null
scp @ScpOpt "${Target}:/tmp/$Csv" "$Results\"
if ($LASTEXITCODE -eq 0) { Write-Host "Saved $Results\$Csv" -ForegroundColor Green }
