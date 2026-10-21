param(
    [string]$WslHost,
    [string]$PythonPath,
    [switch]$Fake
)

$ErrorActionPreference = 'Stop'
$appPath = Join-Path $PSScriptRoot 'app.py'

if (-not (Test-Path -LiteralPath $appPath -PathType Leaf)) {
    throw "Console application not found: $appPath"
}
if ([string]::IsNullOrWhiteSpace($PythonPath)) {
    $pythonCommand = Get-Command python.exe -ErrorAction SilentlyContinue
    if (-not $pythonCommand) {
        $pythonCommand = Get-Command py.exe -ErrorAction SilentlyContinue
    }
    if (-not $pythonCommand) {
        throw 'Python 3.10 or later was not found. Pass -PythonPath.'
    }
    $PythonPath = $pythonCommand.Source
}
if (-not (Test-Path -LiteralPath $PythonPath -PathType Leaf)) {
    throw "Python executable was not found: $PythonPath"
}
$isLauncher = (Split-Path -Leaf $PythonPath) -ieq 'py.exe'
if ($isLauncher) {
    $versionText = & $PythonPath -3 -c "import sys; print('%s.%s' % (sys.version_info[0], sys.version_info[1]))"
} else {
    $versionText = & $PythonPath -c "import sys; print('%s.%s' % (sys.version_info[0], sys.version_info[1]))"
}
if ($LASTEXITCODE -ne 0 -or $versionText -notmatch '^(\d+)\.(\d+)$' -or
    [int]$Matches[1] -lt 3 -or ([int]$Matches[1] -eq 3 -and [int]$Matches[2] -lt 10)) {
    throw "The console needs Python 3.10 or later; '$PythonPath' reported '$versionText'."
}

if ($Fake) {
    if ($isLauncher) {
        & $PythonPath -3 $appPath --fake
    } else {
        & $PythonPath $appPath --fake
    }
    exit $LASTEXITCODE
}

if ([string]::IsNullOrWhiteSpace($WslHost)) {
    throw 'Pass -WslHost using the IPv4 address printed by hostname -I in the already-running Ubuntu-22.04 WSL session.'
}
$wslAddress = $null
if (-not [System.Net.IPAddress]::TryParse($WslHost, [ref]$wslAddress) -or
    $wslAddress.AddressFamily -ne [System.Net.Sockets.AddressFamily]::InterNetwork -or
    $wslAddress.Equals([System.Net.IPAddress]::Any) -or
    $wslAddress.Equals([System.Net.IPAddress]::Loopback)) {
    throw "WslHost must be a concrete WSL IPv4 address; received '$WslHost'."
}
$apiUrl = "http://$($wslAddress.IPAddressToString):18771"
if ($isLauncher) {
    & $PythonPath -3 $appPath --wsl-api-url $apiUrl
} else {
    & $PythonPath $appPath --wsl-api-url $apiUrl
}
exit $LASTEXITCODE
