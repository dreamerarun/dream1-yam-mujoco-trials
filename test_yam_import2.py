"""Find the articulation root the URDF importer produced, then wrap it."""
import os
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
import numpy as np  # noqa: E402
from isaacsim.asset.importer.urdf import _urdf  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from pxr import Usd, UsdPhysics  # noqa: E402

URDF = os.path.expanduser("~/dream-env/lib/python3.12/site-packages/i2rt/robot_models/arm/yam/yam.urdf")
OUT = os.path.expanduser("~/dream1_isaac/yam_left.usd")

world = World(stage_units_in_meters=1.0, physics_dt=1 / 60.0, rendering_dt=1 / 60.0)
world.scene.add_default_ground_plane()

status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.merge_fixed_joints = False
cfg.convex_decomp = False
cfg.import_inertia_tensor = True
cfg.fix_base = True
cfg.distance_scale = 1.0
print("config:", status, flush=True)

ok, prim_path = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=URDF,
                                          import_config=cfg, dest_path=OUT)
print("import:", ok, prim_path, flush=True)

_inst = omni.usd.get_context().get_stage().DefinePrim("/World/yam_left", "Xform")
_inst.GetReferences().AddReference(OUT, "/yam_v0")
stage = omni.usd.get_context().get_stage()


def walk(p, depth=0, limit=40):
    n = 0
    for c in p.GetChildren():
        print("  " * depth + str(c.GetPath()) + " [" + c.GetTypeName() + "]", flush=True)
        n += 1
        if n < limit:
            walk(c, depth + 1, limit)


walk(stage.GetPseudoRoot())

roots = [pr.GetPath() for pr in stage.Traverse()
         if pr.HasAPI(UsdPhysics.ArticulationRootAPI)]
print("articulation roots:", roots, flush=True)

world.reset()
for r in roots:
    try:
        art = world.scene.add(SingleArticulation(prim_path=str(r), name="yam_left"))
        art.initialize()
        print("DOF", art.num_dof, art.dof_names, flush=True)
        print("q", np.round(np.asarray(art.get_joint_positions(), float), 3).tolist(), flush=True)
        break
    except Exception as e:  # noqa: BLE001
        print("wrap failed for", r, repr(e), flush=True)

app.close()
