from setuptools import setup

package_name = 'test_uomo'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/test_uomo.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/test_uomo.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/Pioneer2.urdf',
                                                           'resource/Prob3.urdf',
                                                           'resource/TiagoBase.urdf',
                                                           'resource/percorso_curvo.obj',
                                                           'resource/smalloil.obj',
                                                           'resource/Uomo.urdf'
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
            'pioneer3_driver= test_uomo.pioneer2_driver:main',
            'tiago_base_driver= test_uomo.tiago_base_driver:main',
            'tiago_line_follower = test_uomo.tiago_line_follower:main',    
            'illuminazione = test_uomo.tiago_illuminazione:main',
            'porta = test_uomo.nodoPorta:main',
            'uomo_driver = test_uomo.uomo_driver:main '
    ],
        
    },
)