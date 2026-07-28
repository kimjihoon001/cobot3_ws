import os
from glob import glob

from setuptools import setup

package_name = "monitor_ui"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey",
    maintainer_email="pfr0213@gmail.com",
    description=(
        "트랙 D - 4분할 CCTV형 모니터링 UI: 영상 스트리밍, 상태 집계, 녹화 제어"
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ui_status_node = monitor_ui.ui_status_node:main",
            "recorder_node = monitor_ui.recorder_node:main",
            "qos_bridge = monitor_ui.qos_bridge:main",
            "market_price_node = monitor_ui.market_price_node:main",
        ],
    },
)
