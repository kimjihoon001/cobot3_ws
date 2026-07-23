import os
from glob import glob

from setuptools import setup

package_name = "fleet_dispatch"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "behavior_trees"),
         glob("behavior_trees/*.xml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey",
    maintainer_email="pfr0213@gmail.com",
    description="트랙 B - 운반 배차: fleet_dispatch_node + nav2/AMCL 런치",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fleet_dispatch_node = fleet_dispatch.fleet_dispatch_node:main",
        ],
    },
)
