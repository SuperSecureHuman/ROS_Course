from setuptools import setup

package_name = 'my_py_pkg'

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
    maintainer='SuperSecureHuman',
    maintainer_email='supersecurehuman@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "first_node=my_py_pkg.first_node:main",
            "first_node_oop=my_py_pkg.first_node_oop:main",
            "robo_station=my_py_pkg.robot_news_station:main",
            "smartphone=my_py_pkg.smartphone:main",
            "add_2_client=my_py_pkg.add_two_ints_client:main",
            "add_2_server=my_py_pkg.add_two_ints_server:main",
            "countdown_client=my_py_pkg.countdown_action_client:main",
            "countdown_server=my_py_pkg.countdown_action_server:main",
            "number_counter=my_py_pkg.number_counter:main",
            "num_gen=my_py_pkg.number_gen:main",
            "number_publisher=my_py_pkg.number_publisher_param:main"

        ],
    },

)
