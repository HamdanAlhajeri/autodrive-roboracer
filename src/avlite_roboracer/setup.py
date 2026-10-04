from setuptools import find_packages, setup

setup(
    name="avlite_roboracer",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/avlite_roboracer"]),
        ("share/avlite_roboracer", ["package.xml"]),
        ("share/avlite_roboracer/launch", ["launch/roboracer.launch.py"]),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "avlite-roboracer = avlite_roboracer.cli:main",
            "roboracer_supervisor = avlite_roboracer.runtime:main",
            "roboracer_actuator = avlite_roboracer.actuator_node:main",
            "roboracer_record = avlite_roboracer.recorder:main",
            "roboracer_report = avlite_roboracer.report:main",
        ]
    },
)
