from setuptools import setup

package_name = 'oracle_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='amaraly304@gmail.com',
    description='YOLO-agnostic oracle detector for controller benchmarking',
    license='MIT',
    entry_points={
        'console_scripts': [
            'oracle_detector_node = oracle_detector.oracle_detector_node:main',
        ],
    },
)