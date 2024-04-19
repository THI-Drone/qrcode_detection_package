from setuptools import find_packages, setup

package_name = 'qrcode_detection_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python3-opencv', 'common_package_py'],
    zip_safe=True,
    maintainer='Finn',
    maintainer_email='fie6449@thi.de',
    description='This package contains all nodes responsible for detecting QR codes from the camera input',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_code_scanner_node = qrcode_detection_package.main:main',
        ],
    },
)
