from setuptools import find_packages, setup
import os
from glob import glob

package_name = "urc_intelsys_2024"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.[pxy][yma]*")),
        ),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SJSU Robotics Team",
    maintainer_email="christopher.e.hall@sjsu.edu",
    description="URC Intelsys 2024 Launch Files",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={},  # no entry points, only launch files
)
