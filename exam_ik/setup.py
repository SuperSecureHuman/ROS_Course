from setuptools import setup

package_name = 'exam_ik'

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
    maintainer='root',
    maintainer_email='supersecurehuman@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ik_2_client = exam_ik.ik_2_client:main",
            "ik_2_server = exam_ik.ik_2_server:main",
            "ik_3_server = exam_ik.ik_3_server:main",
            "ik_3_client = exam_ik.ik_3_client:main"
        ],
    },
)
