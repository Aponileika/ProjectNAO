from pathlib import Path

from setuptools import setup

package_name = "cascar"
submodule_1 = "cascar/sensors"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, submodule_1],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            str(Path("share") / package_name / "launch"),
            list(map(str, Path("launch").glob("*"))),
        ),
        (
            str(Path("share") / package_name / "model"),
            list(map(str, Path("model").glob("*"))),
        ),
        (
            str(Path("share") / package_name / "rviz"),
            list(map(str, Path("rviz").glob("*"))),
        ),
    ],
    install_requires=["setuptools", "numpy", "smbus2", "serial"],
    zip_safe=True,
    maintainer="west",
    maintainer_email="theodor.westny@liu.se",
    description="TODO: Package description",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cascar = cascar.main:main",
            "imu = cascar.imu:main",
        ],
    },
)
