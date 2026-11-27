from glob import glob

from setuptools import setup

package_name = 'va_manobras'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rotas', glob('rotas/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Docentes de VA',
    maintainer_email='va@ua.pt',
    description='Estrada como referência, ocupação da faixa, manobra de exemplo e simulador 2D',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_estrada = va_manobras.sim_estrada:main',
            'estrada_rota = va_manobras.estrada_rota:main',
            'faixa_livre = va_manobras.faixa_livre:main',
            'seguir = va_manobras.seguir:main',
            'objetivo_estrada = va_manobras.objetivo_estrada:main',
        ],
    },
)
