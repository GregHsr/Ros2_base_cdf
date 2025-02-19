import os
from glob import glob

from setuptools import setup, SetuptoolsDeprecationWarning, find_packages
import warnings

warnings.filterwarnings("ignore", category=SetuptoolsDeprecationWarning)

package_name = 'action_sequencer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('action_sequencer/config/*.*')
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
            'action_sequencer_node = action_sequencer.action_sequencer_node:main'
        ],
    },
)
