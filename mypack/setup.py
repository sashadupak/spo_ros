from setuptools import setup

package_name = 'mypack'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ilvif',
    maintainer_email='ilvif666@gmail.com',
    description='his package shows real coordinates according to the frame of camera',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'show_coords = mypack.pcd_subscriber_node:main',
            'get_depth = mypack.depth:main',
            'get_coords = mypack.draw:main',
            'start = mypack.start:main',
        ],
    },
)
