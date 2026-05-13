from setuptools import find_packages, setup

package_name = "gazebosimulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", ["config/default.yaml"]),
        (f"share/{package_name}/launch", ["launch/guidance.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Anti-Drone",
    maintainer_email="user@example.com",
    description="PX4 offboard guidance bridge reusing the lightweight Python pursuit algorithms.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "guidance_node = gazebosimulation.guidance_node:main",
        ],
    },
)
