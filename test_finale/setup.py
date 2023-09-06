from setuptools import setup

package_name = 'test_finale'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/launch_test_finale.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/test_finale1.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/Pioneer2.urdf',
                                                           'resource/Prob3.urdf',
                                                           'resource/TiagoBase.urdf',
                                                           'resource/Beep.urdf',
                                                           'resource/percorso_curvo.obj',
                                                           'resource/smalloil.obj',
                                                           'resource/beepCojote.obj'
                                                           ]))

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
            'pioneer3_driver= test_finale.pioneer2_driver:main',
            'tiago_base_driver= test_finale.tiago_base_driver:main',
            'tiago_line_follower = test_finale.tiago_line_follower:main',    
            'illuminazione = test_finale.tiago_illuminazione:main',
            'porta = test_finale.nodoPorta:main',
            'beep_driver = test_finale.beep_driver:main',
            'nodo_porta1 = test_finale.client_porta1:main',
            'nodo_porta2 = test_finale.client_porta2:main',
            'line_follower = test_finale.line_follower:main'

            ],
        
    },
)