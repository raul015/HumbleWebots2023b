from setuptools import setup

package_name = 'prob3_pack'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/prob_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/p-rob3.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_robot.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'prob3_driver = prob3_pack.prob3_driver_v3:main',
            'prob3_driver = prob3_pack.prob3_driver_v2:main',

            'nodo_cliente = prob3_pack.NodoClient:main',
            'service_node_prob3 = prob3_pack.ServiceProb3:main',
            'funzioni = prob3_pack.Funzioni:main',
            'publisher_JointPos = prob3_pack.publisher_JointPos:main',
            'publisher_JointVel = prob3_pack.publisher_JointVel:main',
            'nodo_cliente_v3 = prob3_pack.NodoClient_v3:main',
            'subscriber_sensors_values = prob3_pack.subscriber_JointPosition:main',
            ],
    },
)