from setuptools import find_packages, setup
import os
from glob import glob

package_name = "peregrine_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="px4ros2",
    maintainer_email="vaidyavarad2001@gmail.com",
    description="Launch and configuration package for single-UAV PEREGRINE bringup.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
