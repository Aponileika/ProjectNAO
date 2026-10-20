from pathlib import Path

from setuptools import find_packages, setup

package_name = "cascar_examples"

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
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="davax, west",
    maintainer_email="david.axelsson@liu.se, theodor.westny@liu.se",
    description="Manual controller example for Cascar projects.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "manual_controller = cascar_examples.manual_controller:main",
        ],
    },
)
