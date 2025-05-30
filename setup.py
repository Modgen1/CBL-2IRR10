from setuptools import setup

package_name = 'CmdPublisher'

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
    maintainer='ubuntuhost',
    maintainer_email='ubuntuhost@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'vel_publisher = CmdPublisher.vel_publisher:main',
    'publish_subscribe = CmdPublisher.publish_subscribe:main',
    'coordinate_navigation = CmdPublisher.coordinate_navigation:main'
        
        ],
    },
)
