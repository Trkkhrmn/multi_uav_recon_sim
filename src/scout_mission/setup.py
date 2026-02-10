from setuptools import find_packages, setup

package_name = 'scout_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tarik',
    maintainer_email='user@todo.todo',
    description='Leader drone zone transit + optional scan (satellite_scout_node)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'satellite_scout_node = scout_mission.satellite_scout_node:main',
        ],
    },
)
