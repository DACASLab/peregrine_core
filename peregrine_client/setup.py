from setuptools import find_packages, setup

package_name = "peregrine_client"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Varad Vaidya",
    maintainer_email="vaidyavarad2001@gmail.com",
    description="Blocking Python client for the peregrine flight stack.",
    license="MIT",
)
