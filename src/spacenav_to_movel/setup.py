from setuptools import setup

package_name = "spacenav_to_movel"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bejarano",
    maintainer_email="bejarano@jpl.nasa.gov",
    description="Converts Spacenav messages from 3D mouse to URScript MoveL messages.",
    license="Copyright 2024 NASA Jet Propulsion Laboratory",
    entry_points={"console_scripts": ["converter = spacenav_to_movel.converter:main"]},
)
