#Requires -Version 7.3
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$ProjectPath,
    [string]$ComposeService = 'ros2',
    [string]$Image = 'rust-drone-ros2-humble:local',
    [ValidateRange(1, 64)]
    [int]$Jobs = 1
)

$ErrorActionPreference = 'Stop'
$repository = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$project = (Resolve-Path -LiteralPath $ProjectPath).Path
$docker = Get-Command docker -ErrorAction SilentlyContinue
if (-not $docker) {
    $installedDocker = Join-Path $env:ProgramFiles 'Docker/Docker/resources/bin/docker.exe'
    if (Test-Path -LiteralPath $installedDocker) {
        $docker = Get-Item -LiteralPath $installedDocker
    } else {
        throw 'Install Docker Desktop with a supported Linux container backend, then reopen PowerShell.'
    }
}
$dockerPath = if ($docker.Source) { $docker.Source } else { $docker.FullName }
$operatingSystem = & $dockerPath info --format '{{.OSType}}'
if ($LASTEXITCODE -ne 0) { throw 'Start Docker Desktop and wait until its Linux engine is ready.' }
if ($operatingSystem -ne 'linux') { throw 'Switch Docker Desktop to Linux containers.' }

$toolchainMatch = Select-String -LiteralPath (Join-Path $repository 'rust-toolchain.toml') -Pattern '^channel\s*=\s*"([^"]+)"'
if (-not $toolchainMatch) { throw 'Cannot read the Rust toolchain.' }
$toolchain = $toolchainMatch.Matches[0].Groups[1].Value
$target = Join-Path $repository 'target/ros-humble'
New-Item -ItemType Directory -Force -Path $target | Out-Null
foreach ($path in @($repository, $target)) {
    if ($path.Contains(',') -or $path.Contains('"')) { throw "Unsupported Docker mount path: $path" }
}

$compose = @('compose.yaml', 'compose.yml', 'docker-compose.yaml', 'docker-compose.yml') |
    ForEach-Object { Join-Path $project $_ } |
    Where-Object { Test-Path -LiteralPath $_ } |
    Select-Object -First 1
$arguments = @()
if ($compose) {
    $configurationText = & $dockerPath compose --file $compose config --format json
    if ($LASTEXITCODE -ne 0) { throw 'Cannot read the project Compose configuration.' }
    $configuration = ($configurationText -join "`n") | ConvertFrom-Json
    $service = $configuration.services.PSObject.Properties[$ComposeService]
    if (-not $service) { throw "Compose service '$ComposeService' does not exist." }
    if ($service.Value.build) {
        & $dockerPath compose --file $compose build $ComposeService
        if ($LASTEXITCODE -ne 0) { throw 'Cannot build the ROS project image.' }
    }
    $arguments = @('compose', '--file', $compose, 'run', '--rm', '--no-deps', '-T',
        '--volume', "$($repository.Replace('\', '/')):/workspace/ros2-studio:ro",
        '--volume', "$($target.Replace('\', '/')):/workspace/ros2-studio/target/ros-humble")
} else {
    $arguments = @('run', '--rm',
        '--mount', "type=bind,src=$($repository.Replace('\', '/')),dst=/workspace/ros2-studio,readonly",
        '--mount', "type=bind,src=$($target.Replace('\', '/')),dst=/workspace/ros2-studio/target/ros-humble")
}
$arguments += @('--workdir', '/workspace/ros2-studio', '--entrypoint', 'bash',
    '--env', 'CARGO_HOME=/workspace/ros2-studio/target/ros-humble/cargo',
    '--env', 'RUSTUP_HOME=/workspace/ros2-studio/target/ros-humble/rustup')
$arguments += if ($compose) { $ComposeService } else { $Image }
$build = @'
set -eo pipefail
. /opt/ros/humble/setup.bash
. /opt/ros2-rust-overlay/install/setup.bash
set -u
rustup toolchain install "$1" --profile minimal
CARGO_TARGET_DIR=target/ros-humble cargo "+$1" build --locked --release -j "$2" -p ros_studio_daemon --features ros-runtime --bin ros-studio-daemon
'@
$arguments += @('-c', $build.Replace("`r", ''), 'ros-studio-build', $toolchain, "$Jobs")
& $dockerPath @arguments
if ($LASTEXITCODE -ne 0) { throw 'Cannot build the Linux ROS daemon.' }
$binary = Join-Path $target 'release/ros-studio-daemon'
if (-not (Test-Path -LiteralPath $binary)) { throw "Build completed without creating $binary" }
Write-Output "Created $binary"
