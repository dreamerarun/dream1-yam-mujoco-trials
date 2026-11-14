"""Free: search joint poses for a YAM home that reaches over the table with the tool down."""
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import itertools  # noqa: E402

import numpy as np  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.asset.importer.urdf import _urdf  # noqa: E402,F401

URDF = os.path.expanduser("~/dream-env/lib/python3.12/site-packages/i2rt/robot_models/arm/yam/yam.urdf")
OUT = os.path.expanduser("~/dream1_isaac/yam_probe.usd")

world = World(stage_units_in_meters=1.0, physics_dt=1 / 60.0, rendering_dt=1 / 60.0)
status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.merge_fixed_joints = False
cfg.fix_base = True
ok, root = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=URDF, import_config=cfg,
                                     dest_path=OUT)
prim = omni.usd.get_context().get_stage().DefinePrim("/World/yam", "Xform")
prim.GetReferences().AddReference(OUT, root)
world.reset()
art = world.scene.add(SingleArticulation(prim_path="/World/yam", name="yam"))
art.initialize()


def mat_to_quat(M):
    # not needed: we only read the flange axes
    return M


def quat_to_mat(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def probe(q):
    art.set_joint_positions(np.asarray(q, float))
    world.step(render=False)
    pos, quat = art.get_link_pose("link_6")
    Rm = quat_to_mat(np.asarray(quat, float))
    tcp = np.asarray(pos, float) + 0.1347 * Rm[:, 2]
    return tcp, Rm[:, 2]


best = None
for j1, j2, j3, j4 in itertools.product([0.0, 0.4, -0.4], [0.6, 1.0, 1.4, 1.8],
                                        [0.4, 0.9, 1.4, 1.9], [-1.2, -0.6, 0.0]):
    q = [j1, j2, j3, j4, 0.0, 0.0]
    tcp, approach = probe(q)
    # want: over the table (x ~0.30), z ~0.22, tool pointing down (approach z < -0.85)
    score = abs(tcp[0] - 0.30) + abs(tcp[2] - 0.22) + 2.0 * max(0.0, approach[2] + 0.85) + abs(tcp[1]) * 0.2
    if best is None or score < best[0]:
        best = (score, q, tcp, approach)
    print(f"q={np.round(q,2).tolist()} tcp={np.round(tcp,3).tolist()} approach={np.round(approach,2).tolist()} score={score:.3f}",
          flush=True)

print("BEST", np.round(best[1], 3).tolist(), "tcp", np.round(best[2], 3).tolist(),
      "approach", np.round(best[3], 3).tolist(), flush=True)
app.close()
