from setuptools import find_packages, setup, SetuptoolsDeprecationWarning
import warnings
import os
from glob import glob
warnings.filterwarnings("ignore", category=SetuptoolsDeprecationWarning)

package_name = 'big_brother'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'scripts'),
            glob('big_brother/scripts/*.*')
         ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clément Ramirez',
    maintainer_email='ramirez.clement3@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'big_brother_node = big_brother.big_brother_seq:main'
        ],
    },
)
