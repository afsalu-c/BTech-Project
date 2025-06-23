from setuptools import setup
package_name = "odometry"
setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Afsalu",
    maintainer_email="your_email@example.com",
    description="Odometry Python node",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_subscriber = odometry.odom_subscriber:main",
            "ticks_to_odom = odometry.ticks_to_odom:main",
        ],
    },
)
