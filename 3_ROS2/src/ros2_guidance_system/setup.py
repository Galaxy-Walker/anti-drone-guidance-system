import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_guidance_system'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 索引标记
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 文件
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # 配置文件
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='honor',
    maintainer_email='15303332860@163.com',
    description='基于比例导引(PN)算法的反无人机拦截系统 ROS2 包',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pn_guidance_node = ros2_guidance_system.pn_guidance_node:main',
        ],
    },
)
