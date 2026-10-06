from setuptools import find_packages, setup

package_name = "homeagent_safety"

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
    description="Safety policy gate for HomeAgent high-level robot actions.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safety_node = homeagent_safety.safety_node:main",
        ],
    },
)
