#!/usr/bin/env python3
# coding: utf-8

import os
from glob       import glob
from setuptools import find_packages, setup

package_name = 'nicorover2_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.lua')),
        ('share/' + package_name + '/map',    glob('map/*.pbstream')),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sajisaka',
    maintainer_email='s_ajisaka@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
