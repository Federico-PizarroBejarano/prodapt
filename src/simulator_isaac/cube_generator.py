import numpy as np
from omni.isaac.core.objects import DynamicCuboid


def generate_random_cubes(world, num_cubes, y_span=0.6):
    existing_cubes = []

    while num_cubes > 0:
        x, y = -np.random.rand() * 0.45 - 0.55, (np.random.rand() - 0.5) * y_span

        failed = False
        for cube_pos in existing_cubes:
            if np.linalg.norm(np.array([x, y]) - cube_pos) < 0.15:
                failed = True
                continue

        if not failed:
            existing_cubes.append(np.array([x, y]))
            num_cubes -= 1

    for cube_pos in existing_cubes:
        z_angle = np.random.rand()
        add_cube(world, cube_pos[0], cube_pos[1], z_angle)


def generate_cube_setup(world, setup_name, pos_noise=0.0, orient_noise=0.0):
    x_offset = 0.3
    setups = {
        "no-cubes": [],
        "1-cube-flat": [[-0.575, 0, 0]],
        "1-cube-slanted": [[-0.8, 0, 0.5]],
        "2-cube-wall": [[-0.7, 0.075, 0], [-0.7, -0.075, 0]],
        "3-cube-wall": [[-0.575, 0, 0], [-0.575, -0.15, 0], [-0.575, 0.15, 0]],
        "pyramid": [[-0.65, 0, 0], [-0.825, -0.125, 0], [-0.825, 0.125, 0]],
        "1-sided_right": [[-0.575, 0, 0], [-0.4, -0.175, 0]],
        "1-sided_left": [[-0.575, 0, 0], [-0.4, 0.175, 0]],
        "L_right": [
            [-0.575, 0, 0],
            [-0.425, -0.175, 0],
            [-0.275, -0.175, 0],
            [-0.125, -0.175, 0],
        ],
        "L_left": [
            [-0.575, 0, 0],
            [-0.425, 0.175, 0],
            [-0.275, 0.175, 0],
            [-0.125, 0.175, 0],
        ],
        "J_right": [
            [-0.55, 0, 0.15, 1.0, 1.5],
            [-0.425, -0.175, 0],
            [-0.275, -0.175, 0],
            [-0.125, -0.175, 0],
        ],
        "J_left": [
            [-0.55, 0, -0.15, 1.0, 1.5],
            [-0.425, 0.175, 0],
            [-0.275, 0.175, 0],
            [-0.125, 0.175, 0],
        ],
        "deep-bucket": [
            [-0.575, 0, 0],
            [-0.5, -0.15, 0],
            [-0.35, -0.15, 0],
            [-0.5, 0.15, 0],
            [-0.35, 0.15, 0],
        ],
        "bucket": [[-0.575, 0, 0], [-0.4, 0.175, 0], [-0.4, -0.175, 0]],
        "narrow-bucket": [[-0.575, 0, 0], [-0.425, 0.15, 0], [-0.425, -0.15, 0]],
        "wide-bucket": [
            [-0.575, 0.075, 0],
            [-0.575, -0.075, 0],
            [-0.46, 0.175, 0, 0.5, 0.5],
            [-0.46, -0.175, 0, 0.5, 0.5],
        ],
        "V": [
            [-0.625, 0, 0],
            [-0.625 + 0.075 * (2**0.5), 0.075 * (2**0.5), -0.25, 1.5, 1.0],
            [-0.625 + 0.075 * (2**0.5), -0.075 * (2**0.5), 0.25, 1.5, 1.0],
        ],
        "random": [],
        "random_4": [],
        "random_5": [],
    }

    if setup_name == "random":
        generate_random_cubes(world, 3, 0.5)
    elif setup_name == "random_4":
        generate_random_cubes(world, 4, 0.6)
    elif setup_name == "random_5":
        generate_random_cubes(world, 5, 0.7)
    else:
        for cube_pose in setups[setup_name]:
            if len(cube_pose) == 3:
                x_pos, y_pos, orient = cube_pose
                x_scale, y_scale = 1, 1
            else:
                x_pos, y_pos, orient, x_scale, y_scale = cube_pose
            x_pos -= x_offset
            x_pos += np.random.normal(0.0, pos_noise)
            y_pos += np.random.normal(0.0, pos_noise)
            orient += np.random.normal(0.0, orient_noise)

            add_cube(world, x_pos, y_pos, orient, x_scale, y_scale)


def add_cube(world, x_pos, y_pos, orient, x_scale, y_scale):
    idx = int(np.random.rand() * 10000000)
    cube = DynamicCuboid(
        prim_path=f"/World/Cube{idx}",
        name=f"Cube{idx}",
        position=[x_pos, y_pos, 0.075],
        size=0.15,
        scale=[x_scale, y_scale, 1.0],
        mass=20000.0,
        visible=True,
        orientation=[orient, 0, 0, (1.0 - orient**2) ** 0.5],
    )
    cube.set_collision_enabled(True)
    world.scene.add(cube)
