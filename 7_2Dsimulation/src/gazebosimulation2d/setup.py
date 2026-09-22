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
        (f"share/{package_name}/config", ["config/default.yaml", "config/camera_bridge.yaml"]),
        (f"share/{package_name}/launch", ["launch/guidance.launch.py"]),
    ],
    install_requires=["setuptools"],
    # colcon 只有看到 pytest 测试依赖时才用 pytest 运行 test/（否则退化为无参数的 unittest）。
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Anti-Drone",
    maintainer_email="user@example.com",
    description="PX4 offboard guidance bridge for 2D constant-altitude pursuit simulation.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "guidance_node = gazebosimulation2d.guidance_node:main",
            "vision_adapter = gazebosimulation2d.vision_adapter:main",
        ],
    },
)
