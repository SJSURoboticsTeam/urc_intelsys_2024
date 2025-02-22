from setuptools import find_packages, setup
import os
from glob import glob

package_name = "obstacle_detection"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "models"),
            glob("models/**/*.*", recursive=True),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SJSU Robotics Team",
    maintainer_email="Gill.Teghbir@gmail.com",
    description="ROS2 node wrapper for obstacle detection",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_localizer_node = obstacle_detection.camera_localizer_node:main",
        ],
    },
)
