from setuptools import find_packages, setup

package_name = 'build_platform'

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
    maintainer='alvaro',
    maintainer_email='miguel.ler.alonso@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "serial_reader = build_platform.serial_reader:main",
            "arduino_communicator = build_platform.arduino_communicator:main",
            "serial_reader_v2 = build_platform.serial_reader_v2:main",
        ],
    },
)
