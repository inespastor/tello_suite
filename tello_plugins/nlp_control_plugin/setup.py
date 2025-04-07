from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'nlp_control_plugin'

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
    maintainer='ines',
    maintainer_email='ines@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosgpt = nlp_control_plugin.rosgpt:main',
            'rosgptparser_tello = nlp_control_plugin.rosgptparser_tello:main',
            'rosgpt_client_node = nlp_control_plugin.rosgpt_client_node:main',
        ],
    },
)
