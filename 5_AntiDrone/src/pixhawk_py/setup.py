from setuptools import find_packages, setup

package_name = 'pixhawk_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='honor',
    maintainer_email='15303332860@163.com',
    description='PX4 offboard control node migrated from C++ with behavior parity.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'px4_node = pixhawk_py.main:main',
        ],
    },
)
