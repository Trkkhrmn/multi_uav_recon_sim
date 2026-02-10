from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task_allocation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tarik',
    maintainer_email='user@todo.todo',
    description='Multi Robot Task Allocation: assign 6 iris drones to targets and drop',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mrta_node = task_allocation.mrta_node:main',
            'mrta_panel = task_allocation.mrta_panel:main',
        ],
    },
)
