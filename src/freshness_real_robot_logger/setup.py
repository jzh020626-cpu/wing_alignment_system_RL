from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


package_name = "freshness_real_robot_logger"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="codex@example.com",
    description="Minimal ROS2 topic-level calibration logger for real robot network freshness measurements.",
    license="MIT",
    tests_require=["pytest"],
    scripts=[str(Path("scripts") / "synthetic_calibration_publisher.py")],
    entry_points={
        "console_scripts": [
            "calibration_logger = freshness_real_robot_logger.calibration_logger:main",
        ],
    },
)
