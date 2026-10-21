param(
    [Parameter(Mandatory = $true)]
    [string[]]$EpisodeId,

    [Parameter(Mandatory = $true)]
    [ValidateSet('longest_contiguous_valid_policy_grid_window')]
    [string]$WindowPolicy,

    [string]$RunId = "proxybridge_$([DateTime]::UtcNow.ToString('yyyyMMddTHHmmssZ'))"
)

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$Python = 'E:\anaconda3\envs\pi05_lerobot_dataset\python.exe'
if (-not (Test-Path -LiteralPath $Python -PathType Leaf)) {
    throw "Pinned LeRobot Python was not found: $Python"
}
if ($RunId -notmatch '^[A-Za-z0-9][A-Za-z0-9_-]{2,47}$') {
    throw 'RunId must be a safe 3-48 character directory name.'
}

$BridgeRoot = Join-Path $RepoRoot 'cache\training_bridge'
$RuntimeRoot = Join-Path $BridgeRoot "runtime\$RunId"
if (Test-Path -LiteralPath $RuntimeRoot) {
    throw "RunId already has a runtime directory; choose a new RunId: $RuntimeRoot"
}
$HFHome = Join-Path $RuntimeRoot 'hf'
$DatasetCache = Join-Path $RuntimeRoot 'datasets'
$HubCache = Join-Path $RuntimeRoot 'hub'
$TempRoot = Join-Path $RuntimeRoot 'tmp'
New-Item -ItemType Directory -Force -Path `
    $HFHome, $DatasetCache, $HubCache, $TempRoot, `
    (Join-Path $BridgeRoot 'datasets'), `
    (Join-Path $BridgeRoot 'derived'), `
    (Join-Path $BridgeRoot 'reports') | Out-Null

# These variables are set in PowerShell before Python starts.
$env:HF_HOME = $HFHome
$env:HF_DATASETS_CACHE = $DatasetCache
$env:HF_HUB_CACHE = $HubCache
$env:HF_HUB_OFFLINE = '1'
$env:HF_DATASETS_OFFLINE = '1'
$env:HF_HUB_DISABLE_TELEMETRY = '1'
$env:HF_LEROBOT_HOME = Join-Path $BridgeRoot 'datasets'
$env:TEMP = $TempRoot
$env:TMP = $TempRoot
$env:PYTHONDONTWRITEBYTECODE = '1'

$DependencyRoot = Join-Path $RepoRoot 'cache\live_lerobot_deps'
if (Test-Path -LiteralPath $DependencyRoot -PathType Container) {
    if ($env:PYTHONPATH) {
        $env:PYTHONPATH = "$DependencyRoot;$env:PYTHONPATH"
    }
    else {
        $env:PYTHONPATH = $DependencyRoot
    }
}

$Arguments = @(
    '-B', '-m', 'server.training.training_bridge',
    '--run-id', $RunId,
    '--window-policy', $WindowPolicy
)
foreach ($Id in $EpisodeId) {
    $Arguments += @('--episode-id', $Id)
}

Push-Location $RepoRoot
try {
    & $Python @Arguments
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}
finally {
    Pop-Location
}
