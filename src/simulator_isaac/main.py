import argparse
from omni.isaac.kit import SimulationApp


def main(args):
    # Setting up SimulationApp
    simulation_app = SimulationApp({"headless": args.headless})

    from omni.isaac.core.utils.extensions import enable_extension
    from simulator_isaac.ur10e import UR10e
    from simulator_isaac.simulator import Simulator

    enable_extension("omni.isaac.ros2_bridge")
    simulation_app.update()

    # Setting up our simulation
    simulator = Simulator(simulation_app)

    simulator.add_robot(UR10e())

    simulator.setup()
    simulator.world.reset()
    simulator.run(args.random)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--random", action="store_true")
    parser.set_defaults(headless=False)
    parser.set_defaults(randomize_cubes=False)
    args = parser.parse_args()
    main(args)
