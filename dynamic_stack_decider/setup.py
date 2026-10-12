from setuptools import find_packages, setup

package_name = "dynamic_stack_decider"

setup(
    name=package_name,
    version="0.5.3",
    description="Persistent actions selected by reevaluable decision trees",
    python_requires=">=3.10",
    packages=find_packages(exclude=["test", "test.*", "tests", "tests.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["PyYAML>=6"],
    zip_safe=False,
    keywords=["ROS"],
    license="MIT",
    entry_points={},
)
