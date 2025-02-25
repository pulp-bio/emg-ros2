from setuptools import find_packages, setup

package_name = "gapwatch_wrapper"

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
    description="ROS2 wrapper for GAPWatch",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "streamer = gapwatch_wrapper.streamer:main",
            "logger = gapwatch_wrapper.logger:main",
        ],
    },
)
