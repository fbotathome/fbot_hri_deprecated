from setuptools import find_packages, setup
import os

package_name = 'fbot_speech'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='stihl-bolsistas',
    maintainer_email='stihl-bolsistas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_synthesizer = fbot_speech_nodes.speech_synthesizer:main',
            'speech_synthesizer_old = fbot_speech_nodes.speech_synthesizer_old:main',
            'speech_synthesizer_ml = fbot_speech_nodes.speech_synthesizer_ml:main',
            'speech_recognizer = fbot_speech_nodes.speech_recognizer:main',
            'speech_recognizer_old = fbot_speech_nodes.speech_recognizer_old:main',
            'detector_hotword_node = fbot_speech_nodes.detector_hotword_node:main',
            'audio_player = fbot_speech_nodes.audio_player:main',
            ],
    },
    scripts=[
        os.path.join('fbot_speech_scripts', 'wav_to_mouth.py'),
        os.path.join('fbot_speech_scripts', 'detect_hotword.py'),


    ],
)