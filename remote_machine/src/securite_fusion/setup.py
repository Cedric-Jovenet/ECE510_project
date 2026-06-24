from setuptools import find_packages, setup

package_name = 'securite_fusion'

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
    maintainer='ece510',
    maintainer_email='ece510@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fusion_node = securite_fusion.fusion_node:main',
            'camera_raw_node = securite_fusion.camera_raw_node:main',
            'usb_camera_node = securite_fusion.usb_camera_node:main',
            'person_detector_node = securite_fusion.person_detector_node:main',
            'web_viewer_node = securite_fusion.web_viewer_node:main',
            'uwb_serial_node = securite_fusion.uwb_serial_node:main',
            'uwb_udp_node = securite_fusion.uwb_udp_node:main',
            'uwb_position_node = securite_fusion.uwb_position_node:main',
            'uwb_simulator_node = securite_fusion.uwb_simulator_node:main',
            'ultrasonic_node = securite_fusion.ultrasonic_node:main',
            'iot_supervisor_node = securite_fusion.iot_supervisor_node:main',
        ],
    },
)
