from setuptools import find_packages, setup

package_name = "manual_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pygame"],
    zip_safe=True,
    maintainer="west",
    maintainer_email="theodor.westny@liu.se",
    description="TODO: Package description",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "manual_control = manual_control.main:main",
        ],
    },
)
