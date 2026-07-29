from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'embodied_assistant'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raj Thakur',
    maintainer_email='rajgamer743@gmail.com',
    description='Embodied AI Assistant ROS 2 Core',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_navigator = embodied_assistant.semantic_navigator:main',
            'ai_brain = embodied_assistant.ai_brain:main',
            'object_detector = embodied_assistant.object_detector:main',
        ],
    },
)    
    

