from glob import glob

from setuptools import find_packages, setup


package_name = "freshness_real_robot_validation"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="codex@example.com",
    description="Additive three-robot real-system validation package for app-channel-only freshness experiments.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "phase_source_bridge = freshness_real_robot_validation.phase_source_bridge:main",
            "cmd_channel_wrapper = freshness_real_robot_validation.cmd_channel_wrapper:main",
            "shadow_policy_sidecar = freshness_real_robot_validation.shadow_policy_sidecar:main",
            "tx_policy_sidecar = freshness_real_robot_validation.tx_policy_sidecar:main",
            "observe_only_traffic_source = freshness_real_robot_validation.observe_only_traffic_source:main",
            "task_aware_shadow_observer = freshness_real_robot_validation.task_aware_shadow_observer:main",
        ],
    },
)
