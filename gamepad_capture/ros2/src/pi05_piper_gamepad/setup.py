from glob import glob
from setuptools import setup

setup(
    name='pi05_piper_gamepad',
    version='0.1.0',
    packages=['pi05_piper_gamepad'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pi05_piper_gamepad']),
        ('share/pi05_piper_gamepad', ['package.xml']),
        ('share/pi05_piper_gamepad/launch', glob('launch/*.launch.py')),
        ('share/pi05_piper_gamepad/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Pi0.5 project',
    maintainer_email='maintainer@example.invalid',
    description='Xbox/gamepad teleoperation input and capture for PiPER',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gamepad_operator = pi05_piper_gamepad.operator:main',
            'gamepad_input_echo = pi05_piper_gamepad.input_echo:main',
            'gamepad_demo_recorder = pi05_piper_gamepad.recorder:main',
            'gamepad_xinput_ingress = pi05_piper_gamepad.xinput_ingress:main',
        ],
    },
)
