from setuptools import setup

package_name = "state_estimator"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=True,
    maintainer="west",
    maintainer_email="theodor.westny@liu.se",
    description="TODO: Package description",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "state_estimator = state_estimator.main:main",
        ],
    },
)
