param(
    [Parameter(Position = 0)]
    [string]$MachineCodeFile = "",
    [string]$MachineId = "",
    [string]$LicenseId = "",
    [string]$OutputPath = "license.dat"
)

$ErrorActionPreference = "Stop"
$secret = "GCodePostProcessingSystem.LightCommercial.2026"

function Resolve-MachineId([string]$Text) {
    $value = $Text.Trim()

    if ($value -match "([A-Fa-f0-9]{64})") {
        return $matches[1].ToUpper()
    }

    return $value.ToUpper()
}

if ($MachineCodeFile.Trim().Length -gt 0) {
    $MachineId = Get-Content -LiteralPath $MachineCodeFile -Raw
}

$MachineId = Resolve-MachineId $MachineId
$MachineId = $MachineId.Trim()

if ($MachineId.Length -eq 0) {
    throw "MachineId is required. Provide -MachineId or -MachineCodeFile."
}

if ($LicenseId.Trim().Length -eq 0) {
    $LicenseId = "LIC-" + (Get-Date -Format "yyyyMMddHHmmss")
}

if ($OutputPath -eq "license.dat" -and $MachineCodeFile.Trim().Length -gt 0) {
    $OutputPath = Join-Path (Split-Path -Parent (Resolve-Path -LiteralPath $MachineCodeFile)) "license.dat"
}

$payload = "machineId=$MachineId`nlicenseId=$LicenseId"
$bytes = [System.Text.Encoding]::UTF8.GetBytes($payload + $secret)
$sha256 = [System.Security.Cryptography.SHA256]::Create()

for ($round = 0; $round -lt 4096; $round++) {
    $roundBytes = [System.Text.Encoding]::UTF8.GetBytes([string]$round + $secret)
    $combined = New-Object byte[] ($bytes.Length + $roundBytes.Length)
    [Array]::Copy($bytes, 0, $combined, 0, $bytes.Length)
    [Array]::Copy($roundBytes, 0, $combined, $bytes.Length, $roundBytes.Length)
    $bytes = $sha256.ComputeHash($combined)
}

$signatureBytes = $bytes
$signature = -join ($signatureBytes | ForEach-Object { $_.ToString("x2") })

$license = [ordered]@{
    edition = "pro"
    machineId = $MachineId
    licenseId = $LicenseId
    signature = $signature
}

$license | ConvertTo-Json | Set-Content -LiteralPath $OutputPath -Encoding UTF8
Write-Host "Generated license: $OutputPath"
Write-Host "MachineId: $MachineId"
Write-Host "LicenseId: $LicenseId"
