from setuptools import find_packages, setup

package_name = "rosni_robot"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/rosni_robot_bringup.launch.py"],
        ),
        (
            "share/" + package_name + "/launch",
            ["launch/rosni_robot_slam.launch.py"],
        ),
        (
            "share/" + package_name + "/config/rviz2",
            ["config/rviz2/navigation.rviz"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
