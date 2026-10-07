# Configures and builds the analyzer and the decoder tests with MSVC (x64) and Ninja.
#   ./build.ps1 [-Config Release|Debug] [-Test]
param(
    [ValidateSet('Release', 'Debug')] [string] $Config = 'Release',
    [switch] $Test
)
$ErrorActionPreference = 'Stop'

$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vs = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vs) { throw 'Visual Studio with the C++ x64 tools was not found.' }

# VsDevCmd looks vswhere up on PATH.
$env:PATH = "$(Split-Path $vswhere);$env:PATH"
Import-Module (Join-Path $vs 'Common7\Tools\Microsoft.VisualStudio.DevShell.dll')
Enter-VsDevShell -VsInstallPath $vs -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64' | Out-Null

$build = Join-Path $PSScriptRoot "build/$($Config.ToLower())"
cmake -S $PSScriptRoot -B $build -G Ninja "-DCMAKE_BUILD_TYPE=$Config"
if ($LASTEXITCODE) { exit $LASTEXITCODE }
cmake --build $build
if ($LASTEXITCODE) { exit $LASTEXITCODE }
if ($Test) {
    ctest --test-dir $build --output-on-failure
    if ($LASTEXITCODE) { exit $LASTEXITCODE }
}
