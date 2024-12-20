from setuptools import find_packages, setup

package_name = 'drone_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_pg.py']),
        ('share/' + package_name + '/launch', ['launch/launch_test.py']),
        ('share/' + package_name + '/launch', ['launch/launch_s1.py']),
        ('share/' + package_name + '/launch', ['launch/launch_s2.py'])


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pygame_node = drone_ui.pygame_node:main'
        ],
    },
)
