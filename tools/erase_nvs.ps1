param(
    [string]$Port = "COM10",
    [int]$Baud = 460800
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
$EspTool = Join-Path $ProjectRoot ".espressif\esp-idf-v5.4.1\components\esptool_py\esptool\esptool.py"
$Python = Join-Path $env:USERPROFILE ".espressif\python_env\idf5.4_py3.9_env\Scripts\python.exe"

if (-not (Test-Path $Python)) {
    throw "Python environment not found: $Python"
}

if (-not (Test-Path $EspTool)) {
    throw "esptool.py not found: $EspTool"
}

Write-Host "Erasing ESP32-C3 NVS partition on $Port..."
Write-Host "This removes Wi-Fi credentials, Matter fabrics, and commissioning state."

& $Python $EspTool `
    --chip esp32c3 `
    -p $Port `
    -b $Baud `
    --before default_reset `
    --after hard_reset `
    erase_region 0x9000 0x6000

Write-Host "Done. The device should reboot and advertise as MATTER-3840."
