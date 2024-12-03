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
            'speech_synthesizer = nodes.speech_synthesizer:main',
            'speech_synthesizer_old = nodes.speech_synthesizer_old:main',
            'speech_synthesizer_ml = nodes.speech_synthesizer_ml:main',
            'speech_recognizer = nodes.speech_recognizer:main',
            'speech_recognizer_old = nodes.speech_recognizer_old:main',
            'detector_hotword_node = nodes.detector_hotword_node:main',
            'new_detector_hotword_node = nodes.new_detector_hotword_node:main',
            'audio_player = nodes.audio_player:main',
            ],
    },
    scripts=[
        os.path.join('scripts', 'wav_to_mouth.py'),
        os.path.join('scripts', 'detect_hotword.py'),
        os.path.join('scripts', 'new_detect_hotword.py'),


    ],
)