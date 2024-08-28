import numpy as np
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage


class UR10e(SingleManipulator):
    def __init__(self):
        self._prim_path = "/World/UR10e"
        self._ee_prim_name = "flange"

        assets_root_path = get_assets_root_path()
        usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"
        add_reference_to_stage(usd_path=usd_path, prim_path=self._prim_path)

        super().__init__(
            prim_path=self._prim_path,
            name=self._prim_path.split("/")[-1],
            end_effector_prim_name=self._ee_prim_name,
            gripper=None,
        )

        self.set_enabled_self_collisions(True)
        self.pos_reset()

    def pos_reset(self):
        positions = np.array([[0.2515, -2.0226, -2.157, -0.5369, 1.5708, -1.3193]])
        self.set_joint_velocities(velocities=[0, 0, 0, 0, 0, 0])
        self.set_joint_positions(positions=positions)
