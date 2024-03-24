from setuptools import find_packages, setup

package_name = 'data_logger'

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
            "data_emitter = data_logger.data_emitter:main",
            "data_reader = data_logger.data_reader:main",
            "joint_reader = data_logger.joint_reader:main",
            "digital_reader = data_logger.digital_reader:main",
            "analog_reader = data_logger.analog_reader:main",
            "multi_logger = data_logger.multi_logger:main",
            "super_logger = data_logger.super_logger:main",
        ],
    },
)
