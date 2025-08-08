from setuptools import find_packages, setup
from glob import glob

package_name = 'g1_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/urdf", glob('urdf/*')),
        ('share/' + package_name + "/config", ['config/check_joints.rviz']),
        ('share/' + package_name + "/launch", glob('launch/*')),
        ('share/' + package_name + "/meshes", glob('meshes/*')),
        ('share/' + package_name + "/g1_description", glob('g1_description/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ziwen Zhuang',
    maintainer_email='leozhuangzw@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = g1_description.joint_state_publisher:main'
        ],
    },
)
