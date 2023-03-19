from setuptools import setup

package_name = 'store_automation'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/storebot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/store.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/storebot.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Konstantin Petrykin',
    maintainer_email='kpetrykin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'storebot_driver = store_automation.storebot_driver:main',
            'arm_control_action_server = store_automation.arm_control_action_server:main'
        ],
    },
)