# Flash the s3 build to the board from Windows.
#
# The toolchain lives in WSL but the USB device does not: WSL2 has no COM ports,
# and forwarding one needs usbipd plus an elevated shell. Building in WSL and
# flashing from Windows avoids that entirely, at the cost of this script.
#
# NOTE the command is `powershell`, not `pwsh`. This machine has Windows
# PowerShell 5.1; `pwsh` is PowerShell 7 and is not installed.
#
#   powershell -File scripts\flash.ps1               # flash, then open the monitor
#   powershell -File scripts\flash.ps1 -NoMonitor    # flash only
#   powershell -File scripts\flash.ps1 -MonitorOnly  # just watch the log
#   powershell -File scripts\flash.ps1 -Port COM9    # somewhere else
#
# Build first, in WSL:
#   wsl -- bash -lc 'cd /mnt/d/Repos/car/jeep-canbus/jeep-carputer && pio run -e s3'

param(
    [string]$Port = "COM18",
    [string]$Python = "C:\Lang\miniconda3\python.exe",
    [switch]$NoMonitor,
    [switch]$MonitorOnly
)

$ErrorActionPreference = "Stop"
$build = Join-Path $PSScriptRoot "..\.pio\build\s3"

# Offsets are the ESP32-S3 defaults PlatformIO builds against. The app offset
# must match the partition table, which is CONFIG_PARTITION_TABLE_SINGLE_APP:
# factory starts at 0x10000.
$images = @(
    @{ Addr = "0x0";     File = "bootloader.bin" },
    @{ Addr = "0x8000";  File = "partitions.bin" },
    @{ Addr = "0x10000"; File = "firmware.bin"   }
)

$args = @("-m", "esptool", "--port", $Port, "--chip", "esp32s3", "--baud", "921600",
          "write-flash", "-z")
foreach ($img in $images) {
    $path = Join-Path $build $img.File
    if (-not (Test-Path $path)) {
        Write-Error "missing $path -- build in WSL first (see the header of this file)"
    }
    $args += $img.Addr
    $args += $path
}

if (-not $MonitorOnly) {
    Write-Host "flashing $Port from $build" -ForegroundColor Cyan
    & $Python @args
    if ($LASTEXITCODE -ne 0) { Write-Error "esptool failed ($LASTEXITCODE)" }
}

if (-not $NoMonitor) {
    Write-Host "`nopening monitor -- Ctrl+] to quit" -ForegroundColor Cyan
    # The console is native USB-Serial-JTAG, so the baud rate is nominal; any
    # value works. Ctrl+] is miniterm's exit key.
    & $Python -m serial.tools.miniterm --raw --exit-char 29 $Port 115200
}
