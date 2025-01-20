from setuptools import find_packages, setup
import os
from glob import glob

package_name = "maze_runner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        *[
            (os.path.join("share", package_name, os.path.dirname(f)), [f])
            for f in glob("models/**", recursive=True)
            if os.path.isfile(f)
        ],
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="adi",
    maintainer_email="adithya.alokkan@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            'commander_node = maze_runner.commander_node:main',
        ],
    },
)
