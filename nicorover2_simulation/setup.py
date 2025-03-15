#!/usr/bin/env python3
# coding: utf-8

import os
from glob       import glob
from setuptools import find_packages, setup

package_name = 'nicorover2_simulation'

# glob
def glob_recursive(src_dir, pattern='**/*'):
    files = glob(os.path.join(src_dir, pattern), recursive=True)
    return [f for f in files if os.path.isfile(f)]


# create data file
def create_data_files(package_name, directories):
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]

    for directory in directories:
        files = glob_recursive(directory)
        for file in files:
            target_dir = os.path.join('share', package_name, os.path.dirname(file))
            data_files.append((target_dir, [file]))

    return data_files


data_files_dirs = ['launch', 'config', 'worlds', 'models']

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=create_data_files(package_name, data_files_dirs),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='s_ajisaka@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
