import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rtc_teleop_input"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Junho Park",
    maintainer_email="jeryblueput@gmail.com",
    description="Virtual and real gamepad input published as sensor_msgs/Joy.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "teleop_input = rtc_teleop_input.teleop_gui:main",
        ],
    },
)
