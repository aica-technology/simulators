# just a helper function to start the ur5e robot in mujoco

import mujoco
from mujoco import viewer

model = mujoco.MjModel.from_xml_path('../universal_robots_ur5e/scene.xml')
data = mujoco.MjData(model)

viewer.launch(model, data)
