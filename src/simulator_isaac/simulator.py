import numpy as np
import zmq

import omni
from omni.isaac.core import World
import omni.graph.core as og
from omni.isaac.core.utils.prims import get_prim_at_path, get_prim_children

from simulator_isaac.cube_bypass import generate_random_cubes, generate_maze
from simulator_isaac.force_publisher import ForcePublisher


class Simulator:
    def __init__(self, simulation_app):
        self.simulation_app = simulation_app
        self.simulation_app.update()

        self.force_publisher = ForcePublisher()

        self.world = World(
            stage_units_in_meters=1.0, physics_dt=1.0 / 200.0, rendering_dt=1.0 / 50.0
        )
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

    def run(self, randomize_cubes):
        stage = omni.usd.get_context().get_stage()
        tool0_prim = get_prim_at_path("/World/UR10e/tool0")
        world_prim = get_prim_at_path("/World")

        # Socket to receive reset requests
        context = zmq.Context()
        sock = context.socket(zmq.REP)
        sock.bind("tcp://*:5555")

        close = False
        trial_num = 0
        max_trials = 10

        start = np.array([-0.4, 0])
        end = np.array([-1.2, 0])

        while self.simulation_app.is_running():
            if randomize_cubes:
                print("Starting setup")
                generate_random_cubes(self.world, 3)
            else:
                if close:
                    close = False
                    trial_num = 0

                print(f"Starting trial #{trial_num+1}")
                generate_maze(
                    self.world,
                    setup_name="clear" if np.random.randint(0, 10) == 0 else "h-wall",
                )
                trial_num += 1

            self.world.step(render=True)
            self.robot.pos_reset()
            startup_counter = 0
            self.disconnect_controller()
            detection = False
            reset = False
            found_end = False

            while not reset:
                startup_counter += 1
                self.world.step(render=True)

                if startup_counter == 10:
                    self.connect_controller()

                matrix = omni.usd.get_world_transform_matrix(tool0_prim)
                translate = matrix.ExtractTranslation()

                print(np.round(translate[:2], 2))

                measured_forces = self.robot.get_measured_joint_forces()
                ee_force = measured_forces[-3, :]

                if not found_end and np.linalg.norm(translate[:2] - end) < 0.02:
                    found_end = True
                    print("FOUND END")
                if found_end and np.linalg.norm(translate[:2] - start) < 0.02:
                    reset = True
                    print("DONE")
                    ee_force[0] = 1000

                self.force_publisher.publish_force(ee_force)

                prev_detection = detection
                detection = np.linalg.norm(ee_force[3:]) > 1.0
                if detection:
                    x_dir = "LEFT" if ee_force[1] > 0 else "RIGHT"
                    y_dir = "UP" if ee_force[2] > 0 else "DOWN"
                    print(f"{x_dir} ({int(ee_force[1])}), {y_dir} ({int(ee_force[2])})")
                if prev_detection and not detection:
                    print("-------")

                try:
                    message = sock.recv(flags=zmq.NOBLOCK).decode()
                    if message == "reset":
                        reset = True
                        if trial_num >= max_trials:
                            sock.send(b"close")
                            close = True
                        else:
                            sock.send(b"reset")
                    elif message == "close":
                        close = True
                        reset = True
                except:
                    pass

            for prim in get_prim_children(world_prim):
                if "Cube" in prim.GetName():
                    stage.RemovePrim(f"/World/{prim.GetName()}")

        sock.close()
        self.world.stop()
        self.simulation_app.close()

    def disconnect_controller(self):
        self.controller.edit(
            self.graph,
            {
                self.graph_keys.DISCONNECT: [
                    (
                        "SubJointStates.outputs:positionCommand",
                        "ArticulationController.inputs:positionCommand",
                    )
                ]
            },
        )

    def connect_controller(self):
        self.controller.edit(
            self.graph,
            {
                self.graph_keys.CONNECT: [
                    (
                        "SubJointStates.outputs:positionCommand",
                        "ArticulationController.inputs:positionCommand",
                    )
                ]
            },
        )
