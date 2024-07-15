from setuptools import find_packages, setup

package_name = 'example_service_rclpy'

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
    maintainer='tyjt',
    maintainer_email='deqiang.chen@tyjt-ai.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_server_02 = example_service_rclpy.service_server_02:main',
            'service_client_02 = example_service_rclpy.service_client_02:main'
        ],
    },
)
