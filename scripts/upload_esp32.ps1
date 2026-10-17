<#
.SYNOPSIS
    Upload the ESP32 firmware to the CH340 USB-to-serial port Windows currently
    assigned, instead of a hard-coded COM number.

.DESCRIPTION
    PlatformIO is invoked with the port discovered at run time (by CH340 device
    identity, not by "lowest COM number").  If the device is absent or more than
    one USB-to-serial device is present the script fails with actionable
    diagnostics and never uploads to an arbitrary port.

    The esp32dev environment, board, upload speed (921600) and upload protocol
    are unchanged.

.PARAMETER Port
    Optional explicit port (e.g. COM7).  When omitted the port is discovered.

.PARAMETER EnvName
    PlatformIO environment (default esp32dev).

.PARAMETER DryRun
    Discover and print the selected port without uploading.
#>
[CmdletBinding()]
param(
    [string]$Port,
    [string]$EnvName = 'esp32dev',
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'

$projectDir = (Resolve-Path (Join-Path $PSScriptRoot '..\mega2560')).Path
$pio = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\platformio.exe'

# Enumerate serial ports together with their Windows device description so the
# ESP32's USB-to-serial bridge can be identified by name.
function Get-SerialDevices {
    Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -and ($_.Name -match '\((COM\d+)\)') } |
        ForEach-Object {
            $null = $_.Name -match '\((COM\d+)\)'
            [pscustomobject]@{ Port = $Matches[1]; Name = $_.Name }
        } | Sort-Object Port
}

if (-not $Port) {
    $all = @(Get-SerialDevices)

    # The DOIT ESP32 DevKit V1 uses a CH340 USB-to-serial bridge.  Identify it by
    # device identity, never by "lowest COM number".
    $candidates = @($all | Where-Object { $_.Name -match 'CH340' })

    if ($candidates.Count -eq 0) {
        Write-Host '[ERROR] No CH340 USB-to-serial device found. Connect the ESP32 and retry.' -ForegroundColor Red
        Write-Host 'Currently available serial ports:'
        if ($all.Count -gt 0) { $all | ForEach-Object { Write-Host ("  {0,-6} {1}" -f $_.Port, $_.Name) } }
        else { Write-Host '  (none)' }
        exit 2
    }
    if ($candidates.Count -gt 1) {
        Write-Host '[ERROR] Multiple CH340/USB-serial devices found; refusing to guess.' -ForegroundColor Red
        $candidates | ForEach-Object { Write-Host ("  {0,-6} {1}" -f $_.Port, $_.Name) }
        Write-Host 'Disconnect the extra device(s), or re-run with -Port <COMx>.'
        exit 3
    }

    $Port = $candidates[0].Port
    Write-Host ("[INFO] Selected ESP32 upload port {0}  ({1})" -f $Port, $candidates[0].Name) -ForegroundColor Green
}
else {
    $exists = @(Get-SerialDevices) | Where-Object { $_.Port -eq $Port }
    if (-not $exists) {
        Write-Host ("[ERROR] Requested port {0} is not present. Available ports:" -f $Port) -ForegroundColor Red
        $all = @(Get-SerialDevices)
        if ($all.Count -gt 0) { $all | ForEach-Object { Write-Host ("  {0,-6} {1}" -f $_.Port, $_.Name) } } else { Write-Host '  (none)' }
        exit 2
    }
}

if ($DryRun) {
    Write-Host "[DRYRUN] would run: `"$pio`" run -e $EnvName -t upload --upload-port $Port"
    exit 0
}

if (-not (Test-Path $pio)) {
    Write-Host "[ERROR] PlatformIO not found at $pio" -ForegroundColor Red
    exit 5
}

# PlatformIO resolves its project from the current directory, so make sure the
# mega2560 project is used regardless of where the script was launched from.
Push-Location $projectDir
try {
    # Upload.  One bounded retry covers the common case where the HMI or a serial
    # monitor still holds the port for a moment after the operator closes it.
    $maxAttempts = 2
    for ($attempt = 1; $attempt -le $maxAttempts; $attempt++) {
        & $pio run -e $EnvName -t upload --upload-port $Port
        $code = $LASTEXITCODE
        if ($code -eq 0) { exit 0 }
        if ($attempt -lt $maxAttempts) {
            Write-Host "[WARN] Upload failed (exit $code) on $Port. If the port is busy, close the HMI / serial monitor. Retrying once..." -ForegroundColor Yellow
            Start-Sleep -Seconds 2
        }
    }
}
finally {
    Pop-Location
}

Write-Host "[ERROR] Upload failed on $Port. Verify the ESP32 is connected and the port is not open in another program (close the HMI / serial monitor)." -ForegroundColor Red
exit 1
