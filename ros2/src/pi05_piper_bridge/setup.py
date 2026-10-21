from setuptools import setup

package_name = "pi05_piper_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pi0.5 project",
    maintainer_email="maintainer@example.invalid",
    description="Read-only PiPER ROS 2 bridge with bounded management RPC receipts.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "native_manual_gamepad = pi05_piper_bridge.native_manual_gamepad:main",
        "read_only_bridge = pi05_piper_bridge.read_only_bridge:main",
        "native_can_state = pi05_piper_bridge.native_can_state:main",
        "native_camera_metadata = pi05_piper_bridge.native_camera_metadata:main",
        "fake_proposal_receipt_bridge = pi05_piper_bridge.proposal_receipt_bridge:main",
    ]},
)
