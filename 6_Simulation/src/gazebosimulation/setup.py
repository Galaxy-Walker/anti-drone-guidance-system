from setuptools import find_packages, setup

package_name = "gazebosimulation"

# 这是一个 ament_python 包。`data_files` 会把 ROS 元数据、launch 文件和 YAML 配置
# 安装到 package share 目录，使 `ros2 launch` 和 `FindPackageShare("gazebosimulation")`
# 能在 `colcon build` 后找到它们。
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
            # `ros2 run gazebosimulation guidance_node` 会启动桥接节点。
            "guidance_node = gazebosimulation.guidance_node:main",
        ],
    },
)
