"""生成当前解释器的环境证据；不安装依赖、不修改系统。"""
import argparse
import importlib
import json
import platform
from pathlib import Path
import subprocess
import sys


def check(output):
    target=Path(output); target.mkdir(parents=True,exist_ok=True)
    report={'python':sys.version,'executable':sys.executable,'platform':platform.platform(),
            'architecture':platform.machine(),'packages':{},'python_requirement_met':sys.version_info>=(3,10)}
    for name in ['numpy','cv2','matplotlib','yaml','open3d','scipy']:
        try:
            module=importlib.import_module(name)
            report['packages'][name]={'version':getattr(module,'__version__','unknown'),'import':'ok'}
        except Exception as error:
            report['packages'][name]={'import':'failed','error':str(error)}
    report['all_imports_passed']=all(p['import']=='ok' for p in report['packages'].values())
    (target/'environment.json').write_text(json.dumps(report,indent=2))
    # uv创建的虚拟环境可能没有pip模块：使用uv pip freeze --python读取实际环境。
    freeze=subprocess.run([sys.executable,'-m','pip','freeze'],capture_output=True,text=True)
    if freeze.returncode:
        uv=Path.home()/'.local/bin/uv'
        if uv.exists(): freeze=subprocess.run([str(uv),'pip','freeze','--python',sys.executable],capture_output=True,text=True)
    if freeze.returncode==0: (target/'requirements-observed.txt').write_text(freeze.stdout)
    else: (target/'freeze_error.txt').write_text(freeze.stderr)
    print(json.dumps(report,indent=2))
    if not report['python_requirement_met'] or not report['all_imports_passed']: raise SystemExit(1)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--output',default='q1_env/results')
    check(p.parse_args().output)
