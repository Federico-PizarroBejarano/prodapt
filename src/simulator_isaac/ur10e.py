import numpy as np
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.manipulators.grippers import SurfaceGripper
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

        gripper_usd = assets_root_path + "/Isaac/Robots/UR10/Props/short_gripper.usd"
        add_reference_to_stage(
            usd_path=gripper_usd, prim_path=f"{self._prim_path}/{self._ee_prim_name}"
        )
        gripper = SurfaceGripper(
            end_effector_prim_path=f"{self._prim_path}/{self._ee_prim_name}"
        )

        super().__init__(
            prim_path=self._prim_path,
            name=self._prim_path.split("/")[-1],
            end_effector_prim_name=self._ee_prim_name,
            gripper=gripper,
        )

        self.set_enabled_self_collisions(True)
        self.pos_reset()

    def pos_reset(self):
        positions = np.array([[0.2948, -1.4945, -2.1491, -1.0693, 1.5693, -1.2760]])
        self.set_joint_positions(positions=positions)
