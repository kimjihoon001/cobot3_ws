from setuptools import setup

package_name = "color_detector"

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
    maintainer="jihoonkim",
    maintainer_email="kimjihoon001@gmail.com",
    description="Isaac Sim /rgb 를 받아 HSV 로 파랑/초록 큐브를 판별해 /color_id(1/2) 발행",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "color_detector = color_detector.color_detector_node:main",
        ],
    },
)
