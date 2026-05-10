param(
    [string]$RequestFile = "",
    [string]$MachineId = "",
    [Parameter(Mandatory = $true)]
    [string]$Customer,
    [ValidateSet("lite", "pro")]
    [string]$Edition = "pro",
    [string]$Expires = "",
    [string]$LicenseId = "",
    [string]$OutputPath = "license.dat"
)

$ErrorActionPreference = "Stop"
$secret = "GCodePostProcessingSystem.LightCommercial.2026"

if ($RequestFile.Trim().Length -gt 0) {
    $request = Get-Content -LiteralPath $RequestFile -Raw | ConvertFrom-Json
    $MachineId = [string]$request.machineId
}

$MachineId = $MachineId.Trim()

if ($MachineId.Length -eq 0) {
    throw "MachineId is required. Provide -RequestFile or -MachineId."
}

if ($LicenseId.Trim().Length -eq 0) {
    $LicenseId = "LIC-" + (Get-Date -Format "yyyyMMddHHmmss")
}

$payload = "customer=$Customer`nedition=$Edition`nexpires=$Expires`nmachineId=$MachineId`nlicenseId=$LicenseId"
$bytes = [System.Text.Encoding]::UTF8.GetBytes($payload + $secret)
$sha256 = [System.Security.Cryptography.SHA256]::Create()
$signatureBytes = $sha256.ComputeHash($bytes)
$signature = -join ($signatureBytes | ForEach-Object { $_.ToString("x2") })

$license = [ordered]@{
    customer = $Customer
    edition = $Edition
    expires = $Expires
    machineId = $MachineId
    licenseId = $LicenseId
    signature = $signature
}

$license | ConvertTo-Json | Set-Content -LiteralPath $OutputPath -Encoding UTF8
Write-Host "Generated license: $OutputPath"
Write-Host "MachineId: $MachineId"
Write-Host "LicenseId: $LicenseId"
