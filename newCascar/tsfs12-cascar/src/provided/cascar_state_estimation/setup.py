from setuptools import setup

package_name = "cascar_state_estimation"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="west",
    maintainer_email="theodor.westny@liu.se",
    description="Extended Kalman filter state estimation for Cascar.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "state_estimator = cascar_state_estimation.estimator_node:main",
        ],
    },
)
