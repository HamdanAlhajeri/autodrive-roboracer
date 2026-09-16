from setuptools import find_packages, setup

setup(
    name="avlite_autodrive",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/avlite_autodrive"]),
        ("share/avlite_autodrive", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "avlite_runner = avlite_autodrive.runner:main",
            "actuator_adapter = avlite_autodrive.adapter:main",
            "record_lap = avlite_autodrive.record:main",
        ]
    },
)
