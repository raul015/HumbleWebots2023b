from setuptools import setup

package_name = 'olio_test'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/test_olio_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/scivolamento_robot.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/Pioneer2.urdf','resource/Prob3.urdf','resource/TiagoBase.urdf','resource/nuovo_track.obj','resource/nuovo_track_bianco.obj','resource/percorso_curvo.obj','resource/smalloil.obj']))
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
            'prob3_driver = olio_test.prob3_driver:main',
            'pioneer3_driver= olio_test.pioneer2_driver:main',
            'tiago_base_driver= olio_test.tiago_base_driver:main',
            'tiago_line_follower = olio_test.tiago_line_follower:main',
            'plot = olio_test.plot:main',
            ],
        
    },
)