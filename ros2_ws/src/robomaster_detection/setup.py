import os
from glob import glob

from setuptools import find_packages, setup

package_name = "robomaster_detection"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ardusa",
    maintainer_email="ardusa05@gmail.com",
    description="COCO object detection and debug overlay for the RoboMaster EP.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "object_detector_node = robomaster_detection.object_detector_node:main",
            "detection_overlay_node = robomaster_detection.detection_overlay_node:main",
        ],
    },
)
