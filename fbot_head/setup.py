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
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='rich',
    maintainer_email='richardjcassis@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emotions_bridge = fbot_head.emotions_bridge.emotions_bridge:main',
            'emotions_publisher = fbot_head.emotions_publisher:main',
            'neck_controller = fbot_head.neck_controller.neck_controller:main'
        ],
    },
)
