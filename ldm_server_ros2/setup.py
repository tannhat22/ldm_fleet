import os
from glob import glob
from setuptools import find_packages, setup

package_name = "ldm_server_ros2"
submodules = "ldm_server_ros2/mcprotocol"

setup(
    name=package_name,
    version="0.1.0",
    # packages=find_packages(exclude=["test"]),
    packages=[package_name, submodules],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["config.yaml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tannhat",
    maintainer_email="nguyentannhat2298@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lift_server = ldm_server_ros2.lift_server:main",
            "lift_state_update = ldm_server_ros2.lift_state_update:main",
        ],
    },
)
