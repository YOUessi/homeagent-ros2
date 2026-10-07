from setuptools import find_packages, setup

package_name = "homeagent_skills"

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
    description="High-level robot skill execution adapters for HomeAgent.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mock_skill_executor = homeagent_skills.mock_skill_executor:main",
            "nav2_skill_executor = homeagent_skills.nav2_skill_executor:main",
            "moveit_skill_executor = homeagent_skills.moveit_skill_executor:main",
            "gazebo_contact_pick_executor = homeagent_skills.gazebo_contact_pick_executor:main",
        ],
    },
)
