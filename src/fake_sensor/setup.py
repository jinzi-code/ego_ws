from setuptools import setup

package_name = 'fake_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='The fake_sensor package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_sensor_pub = fake_sensor.fake_sensor_pub:main',
        ],
    },
)
