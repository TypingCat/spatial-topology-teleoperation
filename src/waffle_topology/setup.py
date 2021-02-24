from setuptools import setup

package_name = 'waffle_topology'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fini',
    maintainer_email='jinhong.p.noh@gmail.com',
    description='Semantic teleoperation of wheeled mobile robot: waffle',
    license='GNU General Public License v2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deproject_scan = waffle_topology.deproject_scan:main',
            'generate_topology = waffle_topology.generate_topology:main'
        ],
    },
)
