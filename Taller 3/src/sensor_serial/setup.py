from setuptools import setup

package_name = 'sensor_serial'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erick',
    maintainer_email='you@example.com',
    description='Reto Taller 3: Serial->ROS2->Influx/Grafana con QoS conmutable',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node=sensor_serial.sensor_node:main',
            'processor_node=sensor_serial.processor_node:main',
            'monitor_node=sensor_serial.monitor_node:main',
            'exporter_node=sensor_serial.exporter_node:main',
        ],
    },
)
