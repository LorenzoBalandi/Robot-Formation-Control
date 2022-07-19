from setuptools import setup
from glob import glob

package_name = 'my_bearing_formation'
scripts = ['agent_i','visualizer']

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name, glob('resource/*.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lorenzo',
    maintainer_email='lorenzo.balandi@studio.unibo.it',
    description='ROS package for DAS project 1',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '{1} = {0}.{1}:main'.format(package_name, script) for script in scripts
        ],
    },
)

