import omni
from omni.isaac.core import World
import omni.graph.core as og


class Simulator:
    def __init__(self, simulation_app):
        self.simulation_app = simulation_app
        self.simulation_app.update()

        self.world = World(stage_units_in_meters=1.0)
        physics_context = self.world.get_physics_context()
        physics_context.enable_ccd(True)
        self.world.scene.add_default_ground_plane()
        self.graph_keys = og.Controller.Keys
        self.controller = og.Controller()
        self.graph = "/SimulatorActionGraph"

        self.controller.edit(
            {
                "graph_path": self.graph,
                "evaluator_name": "push",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                self.graph_keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("SimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ],
                self.graph_keys.CONNECT: [
                    (
                        "SimTime.outputs:simulationTime",
                        "PublishClock.inputs:timeStamp",
                    )
                ],
            },
        )

    def add_robot(self, robot):
        self.robot = robot
        self.world.scene.add(robot)
        prim_path = robot._prim_path
        self.controller.edit(
            self.graph,
            {
                self.graph_keys.CREATE_NODES: [
                    (
                        "ArticulationController",
                        "omni.isaac.core_nodes.IsaacArticulationController",
                    ),
                    (
                        "PubJointStates",
                        "omni.isaac.ros2_bridge.ROS2PublishJointState",
                    ),
                    (
                        "SubJointStates",
                        "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
                    ),
                    (
                        "PubTransformTree",
                        "omni.isaac.ros2_bridge.ROS2PublishTransformTree",
                    ),
                ],
                self.graph_keys.CONNECT: [
                    ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("OnTick.outputs:tick", "PubJointStates.inputs:execIn"),
                    ("OnTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("OnTick.outputs:tick", "SubJointStates.inputs:execIn"),
                    ("OnTick.outputs:tick", "PubTransformTree.inputs:execIn"),
                    (
                        "SimTime.outputs:simulationTime",
                        "PubJointStates.inputs:timeStamp",
                    ),
                    (
                        "SimTime.outputs:simulationTime",
                        "PubTransformTree.inputs:timeStamp",
                    ),
                    (
                        "SubJointStates.outputs:jointNames",
                        "ArticulationController.inputs:jointNames",
                    ),
                    (
                        "SubJointStates.outputs:positionCommand",
                        "ArticulationController.inputs:positionCommand",
                    ),
                ],
                self.graph_keys.SET_VALUES: [
                    ("ArticulationController.inputs:robotPath", prim_path),
                    ("PubJointStates.inputs:topicName", "joint_states"),
                    ("PubJointStates.inputs:targetPrim", prim_path),
                    ("SubJointStates.inputs:topicName", "joint_command"),
                    ("PubTransformTree.inputs:parentPrim", f"{prim_path}/base"),
                    ("PubTransformTree.inputs:targetPrims", [f"{prim_path}/tool0"]),
                ],
            },
        )

    def setup(self):
        self.appwindow = omni.appwindow.get_default_app_window()
        self.world.add_physics_callback("a1_advance", callback_fn=self.on_physics_step)

    def on_physics_step(self, step_size) -> None:
        og.Controller.evaluate_sync(self.graph)

    def run(self):
        self.robot.pos_reset()
        while self.simulation_app.is_running():
            self.world.step(render=True)

        self.world.stop()
        self.simulation_app.close()
