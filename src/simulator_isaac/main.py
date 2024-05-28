from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.extensions import enable_extension
from simulator_isaac.ur10e import UR10e
from simulator_isaac.simulator import Simulator

enable_extension("omni.isaac.ros2_bridge")
simulation_app.update()


def main():
    simulator = Simulator(simulation_app)

    simulator.add_robot(
        UR10e(),
    )

    simulator.setup()
    simulator.world.reset()
    simulator.run()


if __name__ == "__main__":
    main()
