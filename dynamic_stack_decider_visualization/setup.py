from setuptools import find_packages, setup

package_name = "dynamic_stack_decider_visualization"

setup(
    name=package_name,
    version="0.2.1",
    description="ROS 2 RQT visualization for the Dynamic Stack Decider",
    python_requires=">=3.10",
    packages=find_packages(exclude=["test", "test.*", "tests", "tests.*"]),
    package_data={package_name: ["py.typed"]},
    data_files=[
        ("share/" + package_name + "/resource", ["resource/StackmachineViz.ui"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["plugin.xml"]),
    ],
    install_requires=["pydot>=3", "PyQt5>=5.15"],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "dsd_gui = " + package_name + ".dsd_visualization_plugin:main",
        ],
    },
)
