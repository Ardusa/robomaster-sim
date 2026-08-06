import os
from glob import glob

from setuptools import find_packages, setup

package_name = "robomaster_dashboard"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (
            os.path.join("share", package_name, "www"),
            [f for f in glob("www/*") if os.path.isfile(f)],
        ),
        (
            os.path.join("share", package_name, "www", "js"),
            [f for f in glob("www/js/*") if os.path.isfile(f)],
        ),
    ],
    install_requires=["setuptools", "aiohttp"],
    zip_safe=True,
    maintainer="Ankur",
    maintainer_email="ardusa05@gmail.com",
    description="Operator web UI for RoboMaster EP teleop.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "dashboard_node = robomaster_dashboard.dashboard_node:main",
        ],
    },
)
