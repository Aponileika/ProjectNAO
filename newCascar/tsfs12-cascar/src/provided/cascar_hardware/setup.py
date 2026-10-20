"""Package configuration for the Cascar hardware package."""

from pathlib import Path

from setuptools import find_packages, setup

package_name = "cascar_hardware"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            str(Path("share") / package_name / "launch"),
            list(map(str, Path("launch").glob("*.py"))),
        ),
    ],
    install_requires=["setuptools", "pyserial", "smbus2"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="west",
    maintainer_email="theodor.westny@liu.se",
    description="Arduino and IMU hardware drivers for Cascar vehicles.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "car_driver = cascar_hardware.main:main",
            "imu_driver = cascar_hardware.main:imu_main",
        ],
    },
)
