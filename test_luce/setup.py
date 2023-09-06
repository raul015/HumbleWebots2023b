from setuptools import setup

package_name = 'test_luce'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/test_luce.py']))
# data_files.append(('share/' + package_name + '/worlds', ['worlds/test_scivolamento_solido.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/test_luce_NoBackGround.wbt']))
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
            'pioneer3_driver= test_luce.pioneer2_driver:main',
            'tiago_base_driver= test_luce.tiago_base_driver:main',
            'tiago_line_follower = test_luce.tiago_line_follower:main',    
            'plotOn = test_luce.plot_on:main',
            'plotOff = test_luce.plot_off:main',
            'plot = test_luce.plot:main',
            'illuminazione = test_luce.tiago_illuminazione:main'
    ],
        
    },
)