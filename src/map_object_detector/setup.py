from setuptools import find_packages, setup

package_name = 'map_object_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='Tarik',
    maintainer_email='user@todo.todo',
    description='Blue target detection and world coords (blue_target_mapper), camera view (camera_view_node)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_view_node = map_object_detector.camera_view_node:main',
            'blue_target_mapper = map_object_detector.blue_target_mapper:main',
        ],
    },
)
