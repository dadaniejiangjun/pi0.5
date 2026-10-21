from setuptools import find_packages, setup


package_name = "pi05_windows_operator_gateway"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/windows_operator.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pi0.5 project",
    maintainer_email="maintainer@example.invalid",
    description="Read-only ROS state and dual-camera HTTP preview with fake-only control proposals.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "windows_operator_gateway = pi05_windows_operator_gateway.gateway_node:main",
        ],
    },
)
