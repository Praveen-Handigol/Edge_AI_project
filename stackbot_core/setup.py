from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'stackbot_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='STACKBOT',
    maintainer_email='STACKBOT@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publish = stackbot_core.camera_publish:main',
            'get_data = stackbot_core.get_data:main',
            'speed_control_without_pid = stackbot_core.speed_control_without_pid:main',
            'model_publish = stackbot_core.model_publish:main'
        ],
    },
)
