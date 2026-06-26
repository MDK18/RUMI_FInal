

from setuptools import find_packages, setup
from glob import glob   # findet alle Dateien per Muster (z.B. *.py)
import os               # zum Zusammenbauen von Pfaden 
# Der Package-Name MUSS exakt mit dem Ordnernamen übereinstimmen!
package_name = 'tsp_digital_twin'

setup(
   
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

   
    # Welche Dateien sollen beim Bauen ins 'install/' kopiert werden?
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # Alle Python-Dateien aus launch/ → install/share/.../launch/
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Alle Dateien aus config/ → install/share/.../config/
       
        (os.path.join('share', package_name, 'config'), glob('config/*')),

        # Alle SUMO-Szenariodateien (.net.xml, .rou.xml, .sumocfg etc.)
        # damit unser TechCampus-Szenario zur Laufzeit auffindbar ist
        (os.path.join('share', package_name, 'scenarios', 'techcampus_tsp'),
            glob('scenarios/techcampus_tsp/*')),
    ],


    install_requires=['setuptools'],
    zip_safe=True,

  
    maintainer='ruben',
    maintainer_email='ruben.geiger@gmx.de',
    description='Traffic Signal Priority Digital Twin - SS2026',
    license='MIT',

    extras_require={
        'test': [
            'pytest',
        ],
    },

   
    entry_points={
        'console_scripts': [
            # Node 1: Liest Fahrzeugdaten aus SUMO via TraCI
            'vehicle_publisher = tsp_digital_twin.vehicle_publisher:main',

            # Node 2: Prioritätslogik (Kernstück des Projekts)
            'tsp_controller = tsp_digital_twin.tsp_controller:main',

            # Node 3: Schaltet Ampel via TraCI in SUMO
            'signal_controller = tsp_digital_twin.signal_controller:main',

            # Node 4: Loggt KPI-Daten in CSV für Auswertung
            'metrics_logger = tsp_digital_twin.metrics_logger:main',
            # Node 5: Dashboard in Terminal (Text-UI)
            'dashboard_node = tsp_digital_twin.dashboard_node:main',
        ],
    },
)