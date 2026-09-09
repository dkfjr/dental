import shutil
from pathlib import Path

from openarm_mujoco.v2 import asset_path, openarm_bimanual_xml

dst = Path(__file__).resolve().parent.parent  # sim/
shutil.copy(openarm_bimanual_xml(), dst / "openarm_bimanual.xml")
shutil.copytree(asset_path("assets"), dst / "assets", dirs_exist_ok=True)
print("복사 완료:", dst)
