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
            "smartphone=my_py_pkg.smartphone:main"
        ],
    },

)
