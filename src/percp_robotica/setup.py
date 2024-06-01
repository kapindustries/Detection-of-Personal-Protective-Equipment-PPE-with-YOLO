from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'percp_robotica'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kap',
    maintainer_email='kevin.potosi@uao.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'nodo_captura_video = percp_robotica.captura_video:main',
        'nodo_ajuste_contraste = percp_robotica.ajuste_contraste:main',
        'nodo_percp_epp = percp_robotica.subscriber:main',
        'nodo_cam_pub = percp_robotica.cam_pub:main',
        
        
        ],
    },
)
