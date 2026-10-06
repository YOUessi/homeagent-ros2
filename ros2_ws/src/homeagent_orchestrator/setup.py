from setuptools import find_packages, setup

package_name = "homeagent_orchestrator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tang Naisheng",
    maintainer_email="devnull@example.com",
    description="Main orchestration layer for HomeAgent-ROS2.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mock_planner = homeagent_orchestrator.mock_planner_node:main",
        ],
    },
)
