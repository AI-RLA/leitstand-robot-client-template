from glob import glob

from setuptools import find_packages, setup

package_name = "leitstand_client_ros2"

setup(
    name=package_name,
    version="0.1.2",
    description="ROS 2 node for the Leitstand client: Nav2 navigation and GNSS pose relay.",
    maintainer="Jannik Jose",
    maintainer_email="jannik.jose@hs-osnabrueck.de",
    license="Apache-2.0",
    packages=find_packages(exclude=["tests", "tests.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    python_requires=">=3.10",
    zip_safe=True,
    entry_points={"console_scripts": ["client = leitstand_client_ros2.node:main"]},
)
