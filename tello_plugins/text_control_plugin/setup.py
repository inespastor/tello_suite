from setuptools import find_packages, setup

package_name = 'text_control_plugin'

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
    description='ROS 2 package for controlling Tello drone',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
	"text_control_plugin = text_control_plugin.text_control_node:main"
    ],
},
)

