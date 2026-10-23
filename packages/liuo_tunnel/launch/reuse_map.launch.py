from pathlib import Path
import runpy
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    root = Path((Path(get_package_share_directory('liuo_tunnel')) / 'project_root.txt').read_text())
    return runpy.run_path(str(root / 'tools/launch_visual.py'))['description'](root, True)
