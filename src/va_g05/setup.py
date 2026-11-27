from glob import glob

from setuptools import setup

package_name = 'va_g05'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # configuração (parâmetros, calibração, montagem das câmaras, RViz); os modelos ONNX ficam em
        # src/va_g05/modelos (VA_G05_DIR no Dockerfile), para não ficarem duas vezes na imagem
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Grupo 05',
    maintainer_email='grupo05@ua.pt',
    description='G5: perceção das câmaras laterais, grelha local e avisos',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'percecao = va_g05.processar_tempo_real:main',
            'visualizador = va_g05.visualizador:main',
            'reprodutor = va_g05.reprodutor:main',
            'resumo_online = va_g05.resumo_online:main',
            'scan_to_grid = va_g05.grelha_scan:main',      # só no simulador da aula 11 (perfil manobra)
        ],
    },
)
