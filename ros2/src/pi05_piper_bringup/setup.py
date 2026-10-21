from glob import glob
from setuptools import setup

package_name = "pi05_piper_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pi0.5 project",
    maintainer_email="maintainer@example.invalid",
    description="Launch and profile files for the Pi0.5 ROS 2 bridge.",
    license="Apache-2.0",
)
