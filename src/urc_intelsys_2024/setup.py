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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="eliot",
    maintainer_email="chrehall68@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fake_gps = urc_intelsys_2024.sensors.gps.fake_gps:main",
            "actual_gps = urc_intelsys_2024.sensors.gps.actual_gps:main",
            "gps_listener = urc_intelsys_2024.sensors.gps.gps_listener:main",
            "actual_compass = urc_intelsys_2024.sensors.compass.actual_compass:main",
            "fake_compass = urc_intelsys_2024.sensors.compass.fake_compass:main",
            "compass_listener = urc_intelsys_2024.sensors.compass.compass_listener:main",
        ],
    },
)
