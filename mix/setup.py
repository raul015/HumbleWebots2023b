from setuptools import setup

package_name = 'mix'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/mix_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/base_robot_2.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/Pioneer2.urdf','resource/Prob3.urdf','resource/TiagoBase.urdf','resource/nuovo_track.obj','resource/nuovo_track_bianco.obj','resource/percorso_curvo.obj']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='raul01597@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
            'prob3_driver = mix.prob3_driver:main',
            'pioneer3_driver= mix.pioneer2_driver:main',
            'tiago_base_driver= mix.tiago_base_driver:main',
            'tiago_obstacle_avoidance = mix.tiago_obstacle_avoidance:main',
            'tiago_line_follower = mix.tiago_line_follower:main',
            'super = mix.super:main' 
            ],
    },
)