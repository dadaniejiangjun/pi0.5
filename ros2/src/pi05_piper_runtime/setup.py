from setuptools import setup

package_name = "pi05_piper_runtime"

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
    description="Read-only Pi0.5 policy shadow inference for ROS 2.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "policy_shadow = pi05_piper_runtime.policy_shadow_node:main",
        "target_preview = pi05_piper_runtime.target_preview_node:main",
        "run_task_action = pi05_piper_runtime.run_task_action_node:main",
    ]},
)
