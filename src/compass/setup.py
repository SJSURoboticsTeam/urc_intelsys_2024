from setuptools import find_packages, setup

package_name = "compass"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SJSU Robotics Team",
    maintainer_email="christopher.e.hall@sjsu.edu",
    description="URC Intelsys 2024 Compass",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "compass = compass.main:main",
            "compass_listener = compass.compass_listener:main",
        ],
    },
)
