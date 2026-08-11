import os
from glob import glob

from setuptools import find_packages, setup

package_name = "fs_autonomy"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rishi",
    maintainer_email="rishisatish@rivianvw.tech",
    description="Formula Student autonomy nodes (sim-agnostic core, eufs_sim2 tooling)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "accel_driver = fs_autonomy.accel_driver:main",
            "skidpad_driver = fs_autonomy.skidpad_driver:main",
            "trackdrive_driver = fs_autonomy.trackdrive_driver:main",
            "cone_viz = fs_autonomy.cone_viz:main",
            "sil_accel = fs_autonomy.sil_accel:main",
            "sil_skidpad = fs_autonomy.sil_skidpad:main",
            "sil_trackdrive = fs_autonomy.sil_trackdrive:main",
        ],
    },
)
