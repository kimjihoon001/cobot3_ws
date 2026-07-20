from setuptools import setup

package_name = "warehouse_dock"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey",
    maintainer_email="pfr0213@gmail.com",
    description="트랙 C - 창고 하역: handoff_node, fork_lift_node",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "handoff_node = warehouse_dock.handoff_node:main",
            "fork_lift_node = warehouse_dock.fork_lift_node:main",
        ],
    },
)
