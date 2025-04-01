from setuptools import find_packages, setup

package_name = 'ros2_web_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn',
        'opencv-python',
        'pytest',
        'requests',
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_web_interface = ros2_web_interface.main:main',
            'test_publishers = test.test_publishers:main',
        ],
    },
)
