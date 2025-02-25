from setuptools import find_packages, setup

package_name = "biowolf16_wrapper"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nihil21",
    maintainer_email="mattia.orlandi21@gmail.com",
    description="ROS2 wrapper for BioWolf16",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "streamer = biowolf16_wrapper.streamer:main",
            "logger = biowolf16_wrapper.logger:main",
        ],
    },
)
