from setuptools import find_packages
from setuptools import setup
import os
from glob import glob

package_name = "svl_tests"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config", "sensors"),
            glob("config/sensors/*"),
        ),
        (
            os.path.join("share", package_name, "config", "waypoints"),
            glob("config/waypoints/*"),
        ),
        (
            os.path.join("share", package_name, "param"),
            glob("param/*"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dax Hawkins",
    maintainer_email="dax@gaiaplatform.io",
    description="Package for running SVL tests",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ims_demo = svl_tests.04_ims_demo:main",
        ],
    },
)
