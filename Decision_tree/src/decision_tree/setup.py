from setuptools import setup, find_packages
import glob

package_name = 'decision_tree'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lin',
    maintainer_email='abcd2006abc@outlook.com',
    description='RoboMaster 强化学习导航决策系统',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'decision_tree = decision_tree.decision_tree:main',
        ],
    },
)
