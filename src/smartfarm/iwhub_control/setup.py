# -*- coding: utf-8 -*-
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "iwhub_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml") + glob("config/*.rviz")),
        (os.path.join("share", package_name, "maps"),
         glob("maps/*.pgm") + glob("maps/*.yaml")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jihoonkim",
    maintainer_email="kimjihoon001@gmail.com",
    description="iw.hub 운반 AMR 베이스 제어 — cmd_vel→차동 바퀴 명령, "
                "joint_states→오도메트리(+TF). Nav2 스택 연결.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # /cmd_vel→바퀴속도(joint_command) + joint_states→odom(+TF). Isaac joint 브리지와 짝.
            "base_node = iwhub_control.base_node:main",
        ],
    },
)
