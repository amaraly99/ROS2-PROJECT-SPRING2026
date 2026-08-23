from setuptools import setup

package_name = 'init_gate'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, package_name + '.profiles'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='amaraly304@gmail.com',
    description='SLAM-init warmup gate — experiment-specific, safe to delete later.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'init_gate_node = init_gate.init_gate_node:main',
        ],
    },
)
