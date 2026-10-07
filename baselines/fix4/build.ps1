param([string]$Toolchain = 'C:\Keil_v5\ARM\ARMCC\bin')
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    New-Item -ItemType Directory -Path 'Objects','Listings' -Force | Out-Null
    # 先清本版最终产物；编译失败时不能误把上一次 HEX 当成本次结果。
    foreach ($outputPath in @('Objects/linefollow_calibrated_fix4.hex','Objects/linefollow_calibrated_fix4.axf','Listings/linefollow_calibrated_fix4.map','build.log')) {
        if (Test-Path -LiteralPath $outputPath) { Remove-Item -LiteralPath $outputPath }
    }
    [xml]$project = Get-Content -LiteralPath 'tracking_square_continuous.uvprojx' -Raw
    $objects = @()
    $log = [Collections.Generic.List[string]]::new()
    foreach ($file in $project.Project.Targets.Target.Groups.Group.Files.File) {
        $ext = [IO.Path]::GetExtension($file.FileName)
        if ($ext -notin '.c','.s') { continue }
        $obj = 'Objects\' + [IO.Path]::GetFileNameWithoutExtension($file.FileName) + '.o'
        Write-Host ('Compiling ' + $file.FileName)
        if ($ext -eq '.s') {
            $output = & (Join-Path $Toolchain 'armasm.exe') --cpu Cortex-M3 --debug -o $obj $file.FilePath 2>&1
        } else {
            $output = & (Join-Path $Toolchain 'armcc.exe') --cpu Cortex-M3 --c99 -O1 --debug --split_sections -DUSE_STDPERIPH_DRIVER -DSTM32F10X_HD -Istart -Ilibrary -Iuser -Isystem -Ihardware -c $file.FilePath -o $obj 2>&1
        }
        $status = $LASTEXITCODE
        foreach ($line in $output) { $log.Add([string]$line); Write-Host $line }
        if ($status -ne 0) { throw ('Compilation failed: ' + $file.FileName) }
        $objects += $obj
    }
    $output = & (Join-Path $Toolchain 'armlink.exe') --cpu Cortex-M3 --scatter firmware.sct --entry Reset_Handler --map --symbols --info sizes --list Listings\linefollow_calibrated_fix4.map -o Objects\linefollow_calibrated_fix4.axf @objects 2>&1
    $status = $LASTEXITCODE
    foreach ($line in $output) { $log.Add([string]$line); Write-Host $line }
    if ($status -ne 0) { throw 'Link failed' }
    & (Join-Path $Toolchain 'fromelf.exe') --i32combined --output Objects\linefollow_calibrated_fix4.hex Objects\linefollow_calibrated_fix4.axf
    if ($LASTEXITCODE -ne 0) { throw 'HEX generation failed' }
    $log.Add('BUILD PASS: Objects\linefollow_calibrated_fix4.hex')
    $log | Set-Content -LiteralPath build.log
    Write-Host 'BUILD PASS: Objects\linefollow_calibrated_fix4.hex'
} finally { Pop-Location }
