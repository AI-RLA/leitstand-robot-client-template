from setuptools import find_packages, setup

package_name = "leitstand_client"

setup(
    name=package_name,
    version="0.1.0",
    description="Robot-side client for the Leitstand: registration, factsheet, missions, state, pose.",
    maintainer="Jannik Jose",
    maintainer_email="jannik.jose@hs-osnabrueck.de",
    license="Apache-2.0",
    packages=find_packages(exclude=["tests", "tests.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "eclipse-zenoh>=1.0.0",
        "pyyaml>=6.0",
        "pydantic>=2.6,<3.0",
        "protobuf>=7.35.1",
        "protovalidate>=1.2",
        "leitstand-robot-contract==0.4.0",
    ],
    python_requires=">=3.10",
    zip_safe=True,
)
