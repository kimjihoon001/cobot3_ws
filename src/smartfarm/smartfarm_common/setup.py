from setuptools import setup

package_name = "smartfarm_common"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey",
    maintainer_email="pfr0213@gmail.com",
    description="트랙 A+B 공동 스켈레톤, 트랙 C가 로직 채움: warehouse_manager_node, logger_node",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "warehouse_manager_node = smartfarm_common.warehouse_manager_node:main",
            "logger_node = smartfarm_common.logger_node:main",
        ],
    },
)
