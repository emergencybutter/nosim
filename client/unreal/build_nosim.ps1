# Builds nosim-ffi for Win64 and copies the header, DLL and import library into the
# Unreal plugin's ThirdParty module. Run from anywhere; rerun after changing the Rust side.
$ErrorActionPreference = 'Stop'
$Repo = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$Dest = Join-Path $PSScriptRoot 'NosimClient\Plugins\Nosim\Source\ThirdParty\NosimLibrary'

Push-Location $Repo
try {
    cargo build -p nosim-ffi --release
    if ($LASTEXITCODE -ne 0) { throw "cargo build failed" }
} finally {
    Pop-Location
}

New-Item -ItemType Directory -Force (Join-Path $Dest 'include'), (Join-Path $Dest 'lib\Win64') | Out-Null
Copy-Item (Join-Path $Repo 'ffi\include\nosim.h') (Join-Path $Dest 'include')
foreach ($f in 'nosim_ffi.dll', 'nosim_ffi.dll.lib', 'nosim_ffi.pdb') {
    Copy-Item (Join-Path $Repo "target\release\$f") (Join-Path $Dest 'lib\Win64')
}
Write-Host "nosim-ffi copied to $Dest"
