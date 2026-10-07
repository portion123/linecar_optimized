"""GCC host/固件语法、配置编译分支与Keil引用；不冒充ARMCC链接。"""
import json
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
FLAGS=['-std=c99','-Wall','-Wextra','-Werror',
       '-DUSE_STDPERIPH_DRIVER','-DSTM32F10X_HD','-Istart','-Ilibrary','-Iuser','-Ihardware','-Isystem']

def main():
    if not shutil.which('gcc'): raise RuntimeError('需要GCC进行本次host/语法检查')
    tree=ET.parse(ROOT/'tracking_square_continuous.uvprojx')
    names=[p.text for p in tree.findall('.//Groups/Group/Files/File/FilePath')]
    paths=[Path(n.replace('\\','/')) for n in names]
    missing=[str(p) for p in paths if not (ROOT/p).is_file()]
    assert not missing,missing
    source=[str(p) for p in paths if p.suffix=='.c']
    application=[p for p in source if not p.startswith(('library/','start/'))]
    vendor=[p for p in source if p not in application]
    # 原字库使用扁平二维初始化，只在包含该字库的OLED单元抑制missing-braces。
    for path in application:
        font_flags=['-Wno-missing-braces'] if path=='hardware/OLED.c' else []
        subprocess.run(['gcc',*FLAGS,*font_flags,'-fsyntax-only',path],cwd=ROOT,check=True)
    # SPL按Cortex-M3的32位地址设计；64位host上的指针/地址宽度提示不是ARM错误。
    # 仅供应商源码抑制这两类提示，应用源码继续使用完整-Werror。
    subprocess.run(['gcc',*FLAGS,'-Wno-int-to-pointer-cast','-Wno-pointer-to-int-cast',
                    '-fsyntax-only',*vendor],cwd=ROOT,check=True)
    print(f'GCC syntax PASS: {len(source)} Keil C units; {len(paths)} project references exist')
    directory=ROOT/'tests/native/variants';directory.mkdir(exist_ok=True)
    config=(ROOT/'hardware/CarConfig.h').read_text()
    branches={
        'debug':{'TRACK_DEBUG':'1'},
        'oled_off':{'TRACK_OLED_ENABLED':'0'},
        'corners_off':{'TRACK_ENABLE_CORNERS':'0'},
        'swapped':{'DRIVE_PAIRS_SWAPPED':'1'},
        'sensor_reversed':{'TRACK_SENSOR_REVERSED':'1'},
        'bench':{'TRACK_BENCH_TEST':'1'},
    }
    for label,overrides in branches.items():
        content=config
        for key,value in overrides.items():
            content=re.sub(r'(#define\s+'+key+r'\s+)\S+',lambda m:m[1]+value,content,count=1)
        header=directory/f'{label}_CarConfig.h';header.write_text(content)
        subprocess.run(['gcc',*FLAGS,'-Itests/native','-O2','-shared','-fPIC','-include',str(header),
                        'tests/native/harness.c','-o',str(directory/(label+'.so'))],cwd=ROOT,check=True)
        subprocess.run(['gcc',*FLAGS,'-include',str(header),'-fsyntax-only','user/main.c'],cwd=ROOT,check=True)
        print('Compile-only PASS: '+label)
    fixed=['LEFT_COUNTS_PER_REV','RIGHT_COUNTS_PER_REV','WHEEL_DIAMETER_MM','AXLE_TRACK_MM',
           'SENSOR_FRONT_OFFSET_MM','TRACK_SENSOR_REVERSED','DRIVE_PAIRS_SWAPPED',
           'LEFT_ENCODER_SIGN','RIGHT_ENCODER_SIGN','LEFT_MOTOR_SIGN','RIGHT_MOTOR_SIGN']
    old=(ROOT/'docs/original_source/hardware/CarConfig.h').read_text(encoding='utf-8-sig')
    for key in fixed:
        token=lambda text:re.search(r'#define\s+'+key+r'\s+(\S+)',text)[1]
        assert token(old)==token(config),(key,token(old),token(config))
    gray=ROOT/'hardware/GraySensor.c'
    assert gray.read_bytes()==(ROOT/'docs/original_source/hardware/GraySensor.c').read_bytes()
    print('Calibration/sign macros and PF0..7 GraySensor source unchanged: PASS')
    (ROOT/'tests/build_checks.json').write_text(json.dumps({'syntax_units':len(source),
        'project_references':len(paths),'compile_only_variants':list(branches),
        'host_warning_exclusions':{'hardware/OLED.c':['missing-braces (original font table)'],
            'library_and_start':['int-to-pointer-cast','pointer-to-int-cast (32-bit MCU addresses on 64-bit host)']},
        'calibration_and_mapping_unchanged':True,'stm32_armcc_build':'NOT RUN'},indent=2))

if __name__=='__main__':main()
