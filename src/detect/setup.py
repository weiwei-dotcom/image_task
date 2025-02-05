from setuptools import setup
from glob import glob
import os

package_name = 'detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ww',
    maintainer_email='728841808@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "yolo = detect.yolo:main"
        ],
    },
)
