from pathlib import Path

from setuptools import find_packages, setup

package_name = "cascar"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            str(Path("share") / package_name / "launch"),
            list(map(str, Path("launch").glob("*.py"))),
        ),
        (
            str(Path("share") / package_name / "config"),
            # Keep data-file paths inside the package for colcon --symlink-install.
            # This source symlink points to the editable workspace configuration.
            ["config/cascar.toml"],
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="davax",
    maintainer_email="david.axelsson@liu.se",
    description="Student-facing launch compositions and configuration for Cascar.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [],
    },
)
