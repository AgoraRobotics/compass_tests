from setuptools import setup

package_name = 'compass_tests'

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
    maintainer='ubuntu',
    maintainer_email='viorels@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rotate = compass_tests.compass:main_rotate',
            'square = compass_tests.compass:main_square',
            'line = compass_tests.compass:main_line',
            'vive_frame = compass_tests.vive_frame_sync:main'
        ],
    },
)
