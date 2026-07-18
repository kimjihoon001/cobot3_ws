from setuptools import setup

package_name = "rgb_camera_monitor"

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
    description="Isaac Sim /rgb 카메라 토픽 수신 감시 및 색상 검출 노드",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rgb_watcher = rgb_camera_monitor.rgb_watcher_node:main",
            "color_id_publisher = rgb_camera_monitor.color_id_node:main",
        ],
    },
)
