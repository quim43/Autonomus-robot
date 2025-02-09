from setuptools import find_packages, setup

package_name = "global_planner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/maps", ["maps/map_cleaned.bmp"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="guiser",
    maintainer_email="sergimuac@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "global_planner_node=global_planner.global_planner:main"
        ],
    },
)
