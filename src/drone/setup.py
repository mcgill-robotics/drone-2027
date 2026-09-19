from glob import glob

from setuptools import find_packages, setup

package_name = "drone"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="McGill Robotics Drone Team",
    description="Drone companion-computer code on PX4 uXRCE-DDS",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Flight checks (the 9-step testing ladder)
            "check_link = drone.flight_checks.check_link:main",
            "check_telemetry = drone.flight_checks.check_telemetry:main",
            "check_setpoints = drone.flight_checks.check_setpoints:main",
            "check_offboard = drone.flight_checks.check_offboard:main",
            "check_arm = drone.flight_checks.check_arm:main",
            "check_hover = drone.flight_checks.check_hover:main",
            "check_goto_gps = drone.flight_checks.check_goto_gps:main",
            "check_gps_movement = drone.flight_checks.check_gps_movement:main",
            "check_lap = drone.flight_checks.check_lap:main",
        ],
    },
)
