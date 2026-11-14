"""Free check: can Isaac import Dream's YAM URDF as a usable articulation?"""
import os
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
import numpy as np  # noqa: E402
from isaacsim.asset.importer.urdf import _urdf  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402

URDF = os.path.expanduser("~/dream-env/lib/python3.12/site-packages/i2rt/robot_models/arm/yam/yam.urdf")

world = World(stage_units_in_meters=1.0, physics_dt=1 / 60.0, rendering_dt=1 / 60.0)
world.scene.add_default_ground_plane()

status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.merge_fixed_joints = False
cfg.convex_decomp = False
cfg.import_inertia_tensor = True
cfg.fix_base = True
cfg.distance_scale = 1.0
cfg.collision_from_visuals = True
print("config:", status, flush=True)

ok, prim_path = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=URDF,
                                          import_config=cfg, dest_path=os.path.expanduser("~/dream1_isaac/yam_left.usd"))
print("import:", ok, prim_path, flush=True)

from isaacsim.core.utils.stage import add_reference_to_stage
add_reference_to_stage(usd_path=os.path.expanduser("~/dream1_isaac/yam_left.usd"),
                       prim_path="/World/yam_left")
world.reset()
art = world.scene.add(SingleArticulation(prim_path="/World/yam_left", name="yam_left"))
art.initialize()
print("dof names:", art.dof_names, flush=True)
print("dof count:", art.num_dof, flush=True)
q = np.asarray(art.get_joint_positions(), float)
print("joint positions:", np.round(q, 3).tolist(), flush=True)
rest = np.array([0.0, 0.5, 0.5, -0.35, 0.0, 0.0])
art.set_joint_positions(rest)
for _ in range(60):
    world.step(render=False)
print("after rest pose:", np.round(np.asarray(art.get_joint_positions(), float), 3).tolist(), flush=True)
print("ee pose:", [np.round(x, 3).tolist() for x in art.end_effector.get_world_pose()] if art.end_effector else None,
      flush=True)
print("OK" if art.num_dof >= 6 else "TOO FEW DOF", flush=True)
app.close()
