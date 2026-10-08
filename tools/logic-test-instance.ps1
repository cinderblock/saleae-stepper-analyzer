# Runs a separate, throwaway Logic 2 instance with the freshly built analyzer loaded, for testing
# through the automation API without touching your everyday Logic session.
#
#   ./tools/logic-test-instance.ps1 [-Config Release|Debug] [-Port 10431] [-LoadFile <capture.sal>] [-Stop]
#
# The instance gets its own profile (--user-data-dir) seeded from your Logic settings, a custom
# analyzer path pointing at a staged copy of the DLL (Logic locks the DLL it loaded, so the build
# output stays writable), device scanning disabled (it never grabs real hardware), and the
# automation server on -Port. Running the script again restarts it with the current build.
param(
    [ValidateSet('Release', 'Debug')] [string] $Config = 'Release',
    [int] $Port = 10431,
    [string] $LoadFile,
    [switch] $Stop
)
$ErrorActionPreference = 'Stop'

$root = Join-Path $env:LOCALAPPDATA 'Temp\logic-stepper-test'
$profileDir = Join-Path $root 'profile'
$stage = Join-Path $root 'analyzers'
$logic = Join-Path $env:ProgramFiles 'Logic\Logic.exe'

# Only processes started with this profile; the everyday instance is never touched.
$running = Get-CimInstance Win32_Process -Filter "Name = 'Logic.exe'" |
    Where-Object { $_.CommandLine -and $_.CommandLine.Contains($profileDir) }
foreach ($process in $running) {
    Stop-Process -Id $process.ProcessId -Force -ErrorAction SilentlyContinue
}
if ($running) {
    Start-Sleep -Seconds 2
}
if ($Stop) {
    return
}

$dll = Join-Path $PSScriptRoot "..\build\$($Config.ToLower())\Analyzers\StepperMotorCoilsAnalyzer.dll"
if (-not (Test-Path $dll)) { throw "Build the analyzer first: $dll not found" }
New-Item -ItemType Directory -Force $profileDir, $stage | Out-Null
Copy-Item $dll $stage -Force

$configPath = Join-Path $profileDir 'config.json'
if (-not (Test-Path $configPath)) {
    $userConfig = Join-Path $env:APPDATA 'Logic\config.json'
    $json = if (Test-Path $userConfig) { Get-Content $userConfig -Raw } else { '{}' }
    $settings = [System.Text.Json.Nodes.JsonNode]::Parse($json)
    $paths = [System.Text.Json.Nodes.JsonArray]::new()
    $paths.Add([System.Text.Json.Nodes.JsonNode]$stage)
    $settings['customAnalyzerPaths'] = $paths
    $settings['automationServerEnabled'] = [System.Text.Json.Nodes.JsonNode]$true
    Set-Content $configPath $settings.ToJsonString() -Encoding utf8NoBOM
}

# Tools that are themselves Electron apps can leave ELECTRON_RUN_AS_NODE set, which makes
# Logic.exe start as plain Node and exit.
Remove-Item Env:ELECTRON_RUN_AS_NODE -ErrorAction SilentlyContinue
$env:SALEAE_DISABLE_DEVICE_SCAN = '1'

$arguments = @("--user-data-dir=`"$profileDir`"", '--automation', '--automationPort', "$Port")
if ($LoadFile) {
    # Opens the capture in the UI (captures loaded through the automation API stay headless).
    $arguments += @('--loadFile', "`"$((Resolve-Path $LoadFile).Path)`"")
}
$process = Start-Process -FilePath $logic -PassThru -ArgumentList $arguments

$deadline = (Get-Date).AddSeconds(60)
while ((Get-Date) -lt $deadline) {
    if ($process.HasExited) { throw "Logic exited with code $($process.ExitCode)" }
    if (Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue) {
        Write-Host "Logic test instance ready: automation on port $Port (pid $($process.Id))"
        return
    }
    Start-Sleep -Milliseconds 500
}
throw "Logic did not open automation port $Port within 60 s"
