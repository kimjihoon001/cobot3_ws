from glob import glob

from setuptools import setup

package_name = "harvest_vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ]
    + [
        ("share/" + package_name, [model_path])
        for model_path in glob("resource/*.pt")
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey",
    maintainer_email="pfr0213@gmail.com",
    description="트랙 A - 수확 파이프라인: vision_node, harvest_fsm_node, tray_manager_node",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vision_node = harvest_vision.vision_node:main",
            "harvest_fsm_node = harvest_vision.harvest_fsm_node:main",
            "tray_manager_node = harvest_vision.tray_manager_node:main",
            "vision_debug_view = harvest_vision.vision_debug_view:main",
            "target_approach_node = harvest_vision.target_approach_node:main",
        ],
    },
)
