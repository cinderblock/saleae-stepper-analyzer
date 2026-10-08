# Installs the built analyzer for everyday use in Logic 2 on Windows.
#
#   ./tools/install.ps1 [-Config Release|Debug] [-Destination <folder>] [-Register]
#
# Copies the DLL to a folder of its own (Logic locks the DLL it has loaded, so loading it straight
# from the build folder would block rebuilds). -Register also adds that folder to Logic's custom
# analyzer paths; Logic must be closed for that, because it rewrites its config while running.
# Without -Register, add the folder yourself under Preferences > Custom Low Level Analyzers.
param(
    [ValidateSet('Release', 'Debug')] [string] $Config = 'Release',
    [string] $Destination = (Join-Path $env:LOCALAPPDATA 'Saleae Logic Analyzers\StepperMotorCoils'),
    [switch] $Register
)
$ErrorActionPreference = 'Stop'

$dll = Join-Path $PSScriptRoot "..\build\$($Config.ToLower())\Analyzers\StepperMotorCoilsAnalyzer.dll"
if (-not (Test-Path $dll)) { throw "Build the analyzer first: $dll not found" }

# Only the everyday Logic matters here; the isolated test instance uses its own profile and copy.
$logic = Get-CimInstance Win32_Process -Filter "Name = 'Logic.exe'" |
    Where-Object { $_.CommandLine -and -not $_.CommandLine.Contains('logic-stepper-test') }

New-Item -ItemType Directory -Force $Destination | Out-Null
try {
    Copy-Item $dll $Destination -Force
}
catch {
    if ($logic) { throw "Close Logic first: it has the installed analyzer loaded. ($($_.Exception.Message))" }
    throw
}
Write-Host "Installed $(Split-Path $dll -Leaf) to $Destination"

if (-not $Register) {
    return
}

if ($logic) { throw 'Close Logic before -Register: it overwrites its config while running.' }

$configPath = Join-Path $env:APPDATA 'Logic\config.json'
if (-not (Test-Path $configPath)) { throw "Logic config not found at $configPath; start Logic once first." }

# JsonNode keeps every other value exactly as it was (ConvertFrom-Json would turn date-like
# strings into dates and write them back differently).
$settings = [System.Text.Json.Nodes.JsonNode]::Parse((Get-Content $configPath -Raw))
$paths = $settings['customAnalyzerPaths']
if ($null -eq $paths) {
    $paths = [System.Text.Json.Nodes.JsonArray]::new()
    $settings['customAnalyzerPaths'] = $paths
}
foreach ($path in $paths) {
    if ($path.GetValue[string]() -eq $Destination) {
        Write-Host 'Already registered in Logic.'
        return
    }
}

# Keep a copy of the config as it was, in case anything about the edit is unwelcome.
Copy-Item $configPath "$configPath.before-stepper-install" -Force
$paths.Add([System.Text.Json.Nodes.JsonNode]$Destination)
$options = [System.Text.Json.JsonSerializerOptions]::new()
$options.WriteIndented = $true
$options.Encoder = [System.Text.Encodings.Web.JavaScriptEncoder]::UnsafeRelaxedJsonEscaping
Set-Content $configPath $settings.ToJsonString($options) -Encoding utf8NoBOM
Write-Host "Registered $Destination in Logic's custom analyzer paths (backup: $configPath.before-stepper-install)"
