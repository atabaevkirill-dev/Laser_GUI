from glob import glob

from setuptools import setup

package_name = 'hexabot_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Kirill Atabaev',
    maintainer_email='atabaevkirill@gmail.com',
    description='Gamepad control of Hexabot.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['teleop_node = hexabot_teleop.teleop_node:main']},
)
