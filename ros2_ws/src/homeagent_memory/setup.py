from setuptools import find_packages, setup

package_name = "homeagent_memory"

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
    maintainer="YOUessi",
    maintainer_email="723911823@qq.com",
    description="SQLite-backed household memory and human-correction service for HomeAgent.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "memory_node = homeagent_memory.memory_node:main",
        ],
    },
)
