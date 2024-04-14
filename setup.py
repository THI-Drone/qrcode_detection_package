from setuptools import find_packages, setup

package_name = 'demo_package_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'common_package_py'],
    zip_safe=True,
    maintainer='philipp',
    maintainer_email='phg6386@thi.de',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo_node = demo_package_py.main:main',
        ],
    },
)
