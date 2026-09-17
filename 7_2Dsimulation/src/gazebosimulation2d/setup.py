from setuptools import find_packages, setup

package_name = "gazebosimulation2d"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # 包级 README 已在文档重构中合并到 7_2Dsimulation/README.md，
        # 这里不要再列出它，否则 setuptools 会因文件缺失而安装失败。
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/default.yaml"]),
        (f"share/{package_name}/launch", ["launch/guidance.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Anti-Drone",
    maintainer_email="user@example.com",
    description="PX4 offboard guidance bridge for 2D constant-altitude pursuit simulation.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "guidance_node = gazebosimulation2d.guidance_node:main",
        ],
    },
)
