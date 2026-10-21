param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9_-]{2,47}$')]
    [string]$RunId
)

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$Python = 'E:\anaconda3\envs\pi05_lerobot_dataset\python.exe'
if (-not (Test-Path -LiteralPath $Python -PathType Leaf)) {
    throw "Pinned LeRobot Python was not found: $Python"
}

$BridgeRoot = Join-Path $RepoRoot 'cache\training_bridge'
$RuntimeRoot = Join-Path $BridgeRoot "runtime\$RunId"
if (Test-Path -LiteralPath $RuntimeRoot) {
    throw "RunId already has an isolated runtime directory; choose a new RunId: $RuntimeRoot"
}
$HFHome = Join-Path $RuntimeRoot 'hf'
$DatasetCache = Join-Path $RuntimeRoot 'datasets'
$HubCache = Join-Path $RuntimeRoot 'hub'
$TempRoot = Join-Path $RuntimeRoot 'tmp'
New-Item -ItemType Directory -Force -Path $HFHome, $DatasetCache, $HubCache, $TempRoot | Out-Null

$env:HF_HOME = $HFHome
$env:HF_DATASETS_CACHE = $DatasetCache
$env:HF_HUB_CACHE = $HubCache
$env:HF_HUB_OFFLINE = '1'
$env:HF_DATASETS_OFFLINE = '1'
$env:HF_HUB_DISABLE_TELEMETRY = '1'
$env:HF_LEROBOT_HOME = Join-Path $BridgeRoot "r4_candidates\$RunId\datasets"
$env:TEMP = $TempRoot
$env:TMP = $TempRoot
$env:PYTHONDONTWRITEBYTECODE = '1'

$Arguments = @(
    '-B', '-u', '-m', 'server.training.r4_training_bridge',
    '--run-id', $RunId
)

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
