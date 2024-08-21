import numpy as np
from omni.isaac.core.objects import DynamicCuboid

base_width = 0.9
base_height = 1.1

box_size = 0.15


def generate_bounding_box(world):
    width = base_width  # + (np.random.rand()-0.5)/2.0
    height = base_height + box_size / 2.0

    add_cube(world, -0.25, 0, 0, x_size=0.5, y_size=width / box_size)  # Ceiling
    add_cube(world, -1.35, 0, 0, x_size=0.5, y_size=width / box_size)  # Floor
    add_cube(
        world,
        (-0.25 - height / 2.0 + box_size / 4.0),
        width / 2.0 + box_size / 4.0,
        0,
        x_size=height / box_size,
        y_size=0.5,
    )  # A wall
    add_cube(
        world,
        (-0.25 - height / 2.0 + box_size / 4.0),
        -width / 2.0 - box_size / 4.0,
        0,
        x_size=height / box_size,
        y_size=0.5,
    )  # Another wall

    return width, height


def generate_maze(world, setup_name):
    bound_width, bound_height = generate_bounding_box(world)

    if setup_name == "clear":
        return
    elif setup_name == "floating-wall":
        width = generate_float_in_interval(box_size, bound_width - box_size * 2)
        x_pos = -generate_float_in_interval(0.4 + box_size, 1.2 - box_size)
        add_cube(world, x_pos, 0, 0, x_size=1.0, y_size=width / box_size)
    elif setup_name == "h-wall":
        width = generate_float_in_interval(
            bound_width / 2.0 + box_size * 0.5, bound_width - box_size
        )
        height = generate_float_in_interval(box_size, bound_height - box_size * 5.0)
        x_pos = -generate_float_in_interval(
            0.4 + box_size / 2.0 + width / 2.0, 1.2 - box_size / 2.0 - width / 2.0
        )
        y_pos = (bound_width - width) / 2.0
        if np.random.randint(0, 2) == 0:
            y_pos *= -1
        add_cube(
            world, x_pos, y_pos, 0, x_size=height / box_size, y_size=width / box_size
        )


def generate_random_cubes(world, num_cubes, y_span=0.6):
    existing_cubes = []

    while num_cubes > 0:
        x, y = -np.random.rand() * 0.45 - 0.55, (np.random.rand() - 0.5) * y_span

        failed = False
        for cube_pos in existing_cubes:
            if np.linalg.norm(np.array([x, y]) - cube_pos) < box_size:
                failed = True
                continue

        if not failed:
            existing_cubes.append(np.array([x, y]))
            num_cubes -= 1

    for cube_pos in existing_cubes:
        z_angle = np.random.rand()
        add_cube(world, cube_pos[0], cube_pos[1], z_angle)


def generate_cube_setup(world, setup_name, pos_noise=0.0, orient_noise=0.0):
    x_offset = 0.2
    setups = {
        "no-cubes": [],
        "1-cube-flat": [[-0.575, 0, 0]],
        # "1-cube-slanted": [[-0.8, 0, 0.5]],
        # "2-cube-wall": [[-0.7, 0.075, 0], [-0.7, -0.075, 0]],
        "3-cube-wall": [[-0.575, 0, 0], [-0.575, -0.15, 0], [-0.575, 0.15, 0]],
        # "pyramid": [[-0.65, 0, 0], [-0.825, -0.125, 0], [-0.825, 0.125, 0]],
        # "1-sided-bucket": [[-0.85, 0, 0], [-0.7, 0.15, 0]],
        "L_right": [[-0.575, 0, 0], [-0.475, -0.175, 0], [-0.325, -0.175, 0]],
        "L_left": [[-0.575, 0, 0], [-0.475, 0.175, 0], [-0.325, 0.175, 0]],
        "deep_bucket": [
            [-0.575, 0, 0],
            [-0.475, -0.175, 0],
            [-0.325, -0.175, 0],
            [-0.475, 0.175, 0],
            [-0.325, 0.175, 0],
        ],
        "2-sided-bucket": [[-0.575, 0, 0], [-0.4, 0.175, 0], [-0.4, -0.175, 0]],
        # "random": [],
        # "random_4": [],
        # "random_5": [],
    }

    if setup_name == "random":
        generate_random_cubes(world, 3, 0.5)
    elif setup_name == "random_4":
        generate_random_cubes(world, 4, 0.6)
    elif setup_name == "random_5":
        generate_random_cubes(world, 5, 0.7)
    else:
        for cube_pose in setups[setup_name]:
            x_pos, y_pos, orient = cube_pose
            x_pos -= x_offset
            x_pos += np.random.normal(0.0, pos_noise)
            y_pos += np.random.normal(0.0, pos_noise)
            orient += np.random.normal(0.0, orient_noise)

            add_cube(world, x_pos, y_pos, orient)


def add_cube(world, x_pos, y_pos, orient, x_size, y_size):
    idx = int(np.random.rand() * 10000000)
    cube = DynamicCuboid(
        prim_path=f"/World/Cube{idx}",
        name=f"Cube{idx}",
        position=[x_pos, y_pos, 0.075],
        size=box_size,
        scale=[x_size, y_size, 0.5],
        mass=20000.0,
        visible=True,
        orientation=[orient, 0, 0, (1.0 - orient**2) ** 0.5],
    )
    cube.set_collision_enabled(True)
    world.scene.add(cube)


def generate_float_in_interval(min_i, max_i):
    i = np.random.rand()
    i *= max_i - min_i
    i += min_i
    return i
