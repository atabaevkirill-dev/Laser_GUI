from setuptools import setup

package_name = 'hexabot_illuminator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kirill Atabaev',
    maintainer_email='atabaevkirill@gmail.com',
    description='Driver for the IR laser illuminator.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': ['illuminator_node = hexabot_illuminator.illuminator_node:main']},
)
