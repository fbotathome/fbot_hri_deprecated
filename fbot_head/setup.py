from setuptools import find_packages, setup
import os
import glob

package_name = 'fbot_head'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rich',
    maintainer_email='richardjcassis@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = fbot_head.my_node:main',
            'emotions_bridge = fbot_head.emotions_bridge.emotions_bridge:main',
            'emotions_publisher = fbot_head.emotions_publisher:main'
        ],
    },
)
