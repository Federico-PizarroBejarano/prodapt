import numpy as np
from omni.isaac.core.objects import DynamicCuboid


def generate_cubes(world, num_cubes):
    existing_cubes = []

    while num_cubes > 0:
        x, y = -np.random.rand() * 0.45 - 0.55, (np.random.rand() - 0.5) * 0.6

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
        idx = int(np.random.rand() * 10000000)
        cube = DynamicCuboid(
            prim_path=f"/World/Cube{idx}",
            name=f"Cube{idx}",
            position=[cube_pos[0], cube_pos[1], 0.075],
            size=0.15,
            mass=20000.0,
            visible=True,
            orientation=[z_angle, 0, 0, (1.0 - z_angle**2) ** 0.5],
        )
        cube.set_collision_enabled(True)
        world.scene.add(cube)
