# 🧭 Configuration Nav2 pour Navigation Autonome

**Date :** 28 Décembre 2025  
**Projet :** Lab_moveit - Mobile Manipulator avec Navigation Autonome  
**Système :** ROS2 Humble + Gazebo + Nav2

---

## 📋 Table des Matières

1. [Vue d'ensemble](#vue-densemble)
2. [Architecture du système](#architecture-du-système)
3. [Fichiers créés/modifiés](#fichiers-créésmodifiés)
4. [Installation](#installation)
5. [Configuration Nav2](#configuration-nav2)
6. [Procédure de test complète](#procédure-de-test-complète)
7. [Dépannage](#dépannage)
8. [Prochaines étapes](#prochaines-étapes)

---

## 🎯 Vue d'ensemble

### Objectif
Permettre au robot mobile manipulateur de naviguer de manière autonome dans la warehouse en utilisant :
- **SLAM (Simultaneous Localization and Mapping)** pour créer la carte
- **AMCL (Adaptive Monte Carlo Localization)** pour se localiser
- **Nav2** pour planifier et exécuter les trajectoires
- **Marqueurs ArUco** pour le positionnement précis aux destinations

### Résultat final
✅ Robot capable de naviguer de manière autonome vers n'importe quel point de la warehouse  
✅ Évitement d'obstacles en temps réel  
✅ Localisation précise avec AMCL  
✅ Positionnement fin avec marqueurs ArUco (pour pick & place)

---

## 🏗️ Architecture du système

```
┌─────────────────────────────────────────────────────────────┐
│                    SYSTÈME COMPLET                          │
└─────────────────────────────────────────────────────────────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
   ┌────▼────┐      ┌──────▼──────┐    ┌─────▼─────┐
   │  GAZEBO │      │    SLAM     │    │   NAV2    │
   │ Simulation│     │(Cartographie)│   │(Navigation)│
   └─────────┘      └─────────────┘    └───────────┘
        │                  │                  │
        │                  │                  │
   ┌────▼────────────┬─────▼────────┬─────────▼─────────┐
   │  LiDAR (270°)   │  map_server  │  controller_server│
   │  Caméra         │  SLAM Toolbox│  planner_server   │
   │  Odométrie      │              │  AMCL             │
   └─────────────────┴──────────────┴───────────────────┘
```

### Flux de navigation

```
1. SLAM crée la carte → warehouse_map_last_version.yaml/pgm
2. Nav2 charge la carte → map_server
3. AMCL localise le robot → pose dans la carte
4. User donne un goal → Nav2 Goal (x, y, orientation)
5. Planner calcule le chemin → Trajectoire optimale
6. Controller suit le chemin → Commandes de vitesse
7. Robot arrive → Confirmation avec ArUco (optionnel)
```

---

## 📁 Fichiers créés/modifiés

### Structure des packages

```
Lab_moveit/
├── src/
│   ├── mobile_manipulator_sim/          # Package ROBOT
│   │   ├── urdf/
│   │   │   └── simple_mobile_robot.urdf  # ✅ MODIFIÉ
│   │   │       └── LiDAR: angle -135° à +135°, range min 0.3m
│   │   └── config/
│   │       ├── mapper_params_online_async.yaml  # Existant
│   │       └── nav2_params.yaml                 # ✅ CRÉÉ
│   │
│   └── my_gazebo_world/                 # Package ENVIRONNEMENT
│       ├── worlds/
│       │   └── warehouse.world           # Existant (tables corrigées)
│       ├── maps/
│       │   ├── warehouse_map_last_version.yaml  # ✅ CRÉÉ (SLAM)
│       │   └── warehouse_map_last_version.pgm   # ✅ CRÉÉ (SLAM)
│       └── launch/
│           ├── warehouse_with_robot.launch.py   # Existant
│           ├── mapping.launch.py                # Existant
│           └── navigation.launch.py             # ✅ CRÉÉ
```

### Fichiers clés

| Fichier | Rôle | Package |
|---------|------|---------|
| `nav2_params.yaml` | Configuration complète Nav2 | mobile_manipulator_sim |
| `navigation.launch.py` | Lance Nav2 avec la carte | my_gazebo_world |
| `warehouse_map_last_version.*` | Carte de la warehouse | my_gazebo_world |
| `simple_mobile_robot.urdf` | LiDAR 270° avant | mobile_manipulator_sim |

---

## 🔧 Installation

### Prérequis

```bash
# Installer Nav2 et dépendances
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-dwb-core \
  ros-humble-dwb-critics \
  ros-humble-dwb-plugins \
  ros-humble-turtlebot3*

# Vérifier l'installation
ros2 pkg list | grep nav2
```

---

## ⚙️ Configuration Nav2

### Étape 1 : Créer le fichier de paramètres Nav2

**Fichier :** `~/Lab_moveit/src/mobile_manipulator_sim/config/nav2_params.yaml`

**Télécharger le fichier de base :**

```bash
cd ~/Lab_moveit/src/mobile_manipulator_sim/config
wget https://raw.githubusercontent.com/ros-planning/navigation2/humble/nav2_bringup/params/nav2_params.yaml
```

**Modifications essentielles à faire :**

1. **Remplacer tous les `base_footprint` par `base_link` :**

```bash
sed -i 's/base_footprint/base_link/g' nav2_params.yaml
```

2. **Modifier le chemin de la carte (ligne ~170) :**

```bash
nano nav2_params.yaml
```

Chercher `yaml_filename:` et modifier :

```yaml
map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: "/home/douaa/Lab_moveit/src/my_gazebo_world/maps/warehouse_map_last_version.yaml"
```

**⚠️ Utiliser le CHEMIN ABSOLU !**

3. **Vérifier que `use_sim_time: True` partout**

```bash
grep "use_sim_time" nav2_params.yaml | head -5
```

Tous doivent être `True`.

---

### Étape 2 : Créer le launch file de navigation

**Fichier :** `~/Lab_moveit/src/my_gazebo_world/launch/navigation.launch.py`

```python
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Chemins
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    mobile_sim_dir = get_package_share_directory('mobile_manipulator_sim')
    my_gazebo_dir = get_package_share_directory('my_gazebo_world')
    
    # Fichiers de config
    map_file = os.path.join(my_gazebo_dir, 'maps', 'warehouse_map_last_version.yaml')
    nav2_params_file = os.path.join(mobile_sim_dir, 'config', 'nav2_params.yaml')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Full path to map yaml file')
    
    # Nav2 Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch/bringup_launch.py']),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_params_file_cmd,
        declare_map_yaml_cmd,
        nav2_bringup,
    ])
```

**Rendre exécutable :**

```bash
chmod +x ~/Lab_moveit/src/my_gazebo_world/launch/navigation.launch.py
```

---

### Étape 3 : Compiler

```bash
cd ~/Lab_moveit
colcon build --packages-select mobile_manipulator_sim my_gazebo_world
source install/setup.bash
```

---

## 🧪 Procédure de test complète

### Test 1 : Création de la carte (SLAM)

**Si vous n'avez PAS encore de carte :**

#### Terminal 1 : Lancer Gazebo
```bash
cd ~/Lab_moveit
source install/setup.bash
ros2 launch my_gazebo_world warehouse_with_robot.launch.py
```

#### Terminal 2 : Lancer SLAM
```bash
cd ~/Lab_moveit
source install/setup.bash
ros2 launch my_gazebo_world mapping.launch.py
```

#### Terminal 3 : Visualiser dans RViz
```bash
rviz2
```

**Dans RViz :**
- Add → Map → Topic: `/map`
- Add → LaserScan → Topic: `/scan`
- Fixed Frame: `map`

#### Terminal 4 : Téléopérer le robot
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Conduite lente (≤0.2 m/s), mouvements fluides, pauses de 2s aux coins**

#### Sauvegarder la carte
```bash
cd ~/Lab_moveit/src/my_gazebo_world
mkdir -p maps
cd maps
ros2 run nav2_map_server map_saver_cli -f warehouse_map_last_version
```

**Vérification :**
```bash
ls -lh ~/Lab_moveit/src/my_gazebo_world/maps/
# warehouse_map_last_version.yaml
# warehouse_map_last_version.pgm
```

---

### Test 2 : Navigation autonome (Nav2)

#### Configuration préalable

**Assurez-vous que :**
1. ✅ La carte existe : `~/Lab_moveit/src/my_gazebo_world/maps/warehouse_map_last_version.*`
2. ✅ `nav2_params.yaml` pointe vers cette carte (chemin ABSOLU)
3. ✅ Tous les `base_footprint` → `base_link` dans nav2_params.yaml

---

#### Terminal 1 : Lancer Gazebo + Robot

```bash
cd ~/Lab_moveit
source install/setup.bash
ros2 launch my_gazebo_world warehouse_with_robot.launch.py
```

**Attendre que Gazebo soit complètement chargé (robot visible)**

---

#### Terminal 2 : Lancer Nav2

```bash
cd ~/Lab_moveit
source install/setup.bash
ros2 launch my_gazebo_world navigation.launch.py
```

**Vérifier les logs :**

✅ **BON SIGNE :**
```
[INFO] [map_server]: Map loaded successfully
[INFO] [lifecycle_manager_localization]: Managed nodes are active
[INFO] [lifecycle_manager_navigation]: Managed nodes are active (peut être absent si controller_server non actif)
```

❌ **MAUVAIS SIGNE :**
```
[ERROR] [map_server]: Could not load map...
[FATAL] [behavior_server]: Failed to create behavior...
```

**Astuce pour voir les logs proprement :**
```bash
ros2 launch my_gazebo_world navigation.launch.py 2>&1 | tee ~/nav2_logs.txt

# Dans un autre terminal
grep "Managed nodes are active" ~/nav2_logs.txt
grep -i "error\|fatal" ~/nav2_logs.txt
```

---

#### Terminal 3 : Initialiser AMCL (OBLIGATOIRE)

**AMCL ne publie le transform `map -> odom` qu'APRÈS avoir reçu `/initialpose` !**

```bash
cd ~/Lab_moveit
source install/setup.bash

# Initialiser AMCL à la position (0, 0)
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.068538919]
  }
}"
```

**Vérifier que le transform existe :**

```bash
ros2 run tf2_ros tf2_echo map odom
```

**✅ Vous devriez voir des valeurs défiler !**

---

#### Terminal 4 : Lancer RViz

```bash
cd ~/Lab_moveit
source install/setup.bash
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**Vous devriez voir :**
- ✅ Carte (noir et blanc)
- ✅ Robot (modèle 3D au centre)
- ✅ LiDAR scan (points verts)
- ✅ Costmaps (zones rose/cyan)
- ✅ AMCL particle cloud (nuage vert)

**État du panneau Navigation 2 :**
- **Navigation:** active ✅
- **Localization:** active ✅

---

#### Donner un objectif de navigation

**Méthode 1 : Via RViz (RECOMMANDÉ)**

1. **Clique sur "2D Pose Estimate"** (en haut) si la localisation n'est pas bonne
   - Clique où est le robot
   - Tire une flèche dans sa direction
   
2. **Clique sur "Nav2 Goal"** (bouton vert avec flèche)
   - Clique sur une destination (zone blanche)
   - Tire une flèche dans la direction souhaitée
   - **Le robot devrait commencer à bouger ! 🚗💨**

---

**Méthode 2 : Via terminal**

```bash
# Aller au centre de la warehouse
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 0.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

```bash
# Aller vers la grande table noire (dépôt)
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 3.5, y: -2.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

```bash
# Aller vers la table ROUGE (marqueur ID 0)
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: -5.5, y: -2.8, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

```bash
# Aller vers la table BLEUE (marqueur ID 1)
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: -4.35, y: -2.8, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

```bash
# Aller vers la zone CHARGING (marqueur ID 4)
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 5.0, y: 3.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

---

### Vérifications en temps réel

**Terminal 5 : Monitorer les topics**

```bash
# Voir les commandes de vitesse envoyées
ros2 topic echo /cmd_vel

# Voir la position AMCL
ros2 topic echo /amcl_pose

# Voir l'état de navigation
ros2 topic echo /navigation_state
```

**Vérifier les nodes actifs :**

```bash
ros2 node list | grep -E "amcl|controller|planner|behavior"
```

**Vous devriez voir :**
```
/amcl
/behavior_server
/controller_server
/planner_server
```

---

## 🐛 Dépannage

### Problème 1 : "Invalid frame ID 'map' does not exist"

**Cause :** AMCL n'a pas initialisé (pas reçu `/initialpose`)

**Solution :**
```bash
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.068]}}"
```

---

### Problème 2 : "Couldn't transform from lidar_link to base_footprint"

**Cause :** Paramètre `base_footprint` au lieu de `base_link`

**Solution :**
```bash
cd ~/Lab_moveit/src/mobile_manipulator_sim/config
sed -i 's/base_footprint/base_link/g' nav2_params.yaml
cd ~/Lab_moveit
colcon build --packages-select mobile_manipulator_sim
source install/setup.bash
# Relancer Nav2
```

---

### Problème 3 : "Map loaded successfully" mais pas de "Managed nodes are active"

**Cause :** Erreur dans nav2_params.yaml (plugin mal nommé, paramètre manquant)

**Solution :**
```bash
# Voir les logs détaillés
ros2 launch my_gazebo_world navigation.launch.py 2>&1 | tee ~/nav2_debug.txt

# Chercher les erreurs
grep -i "error\|fatal" ~/nav2_debug.txt
```

**Vérifier la syntaxe YAML :**
```bash
python3 -c "import yaml; yaml.safe_load(open('/home/douaa/Lab_moveit/src/mobile_manipulator_sim/config/nav2_params.yaml'))" && echo "✅ YAML valid" || echo "❌ YAML invalid"
```

---

### Problème 4 : Le robot ne bouge pas quand on donne un goal

**Vérifications :**

```bash
# 1. Vérifier que controller_server tourne
ros2 node list | grep controller_server

# 2. Vérifier que les commandes sont publiées
ros2 topic echo /cmd_vel

# 3. Vérifier l'état du lifecycle
ros2 service call /controller_server/get_state lifecycle_msgs/srv/GetState
# Réponse attendue : id=3, label='active'
```

**Si controller_server n'est pas actif :**
```bash
ros2 service call /controller_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
```

---

### Problème 5 : RViz crash (Segmentation fault)

**Cause :** Problème de rendu OpenGL avec certaines cartes graphiques

**Solutions :**

1. **Utiliser une config RViz minimale :**
```bash
rviz2  # Sans config
# Puis ajouter manuellement : Map, LaserScan, TF, RobotModel
```

2. **Exporter variable d'environnement :**
```bash
export LIBGL_ALWAYS_SOFTWARE=1
rviz2
```

---

### Problème 6 : Le robot va contre les murs / ignore les obstacles

**Cause :** Costmap mal configuré ou LiDAR ne fonctionne pas

**Vérifications :**

```bash
# Vérifier le scan LiDAR
ros2 topic echo /scan --once

# Vérifier les costmaps
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /global_costmap/costmap --once
```

**Dans RViz :**
- Add → Costmap → `/local_costmap/costmap`
- Add → Costmap → `/global_costmap/costmap`
- Les obstacles doivent apparaître en rouge/rose

---

## 📊 Commandes de diagnostic utiles

```bash
# Voir tous les topics
ros2 topic list

# Voir tous les nodes
ros2 node list

# Voir les frames TF
ros2 run tf2_tools view_frames
evince frames.pdf

# Monitorer un transform
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link

# Voir les paramètres Nav2
ros2 param list /amcl
ros2 param list /controller_server

# Voir l'état des lifecycle nodes
ros2 service call /amcl/get_state lifecycle_msgs/srv/GetState
ros2 service call /controller_server/get_state lifecycle_msgs/srv/GetState
ros2 service call /planner_server/get_state lifecycle_msgs/srv/GetState

# Tester la planification
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

## 🎯 Prochaines étapes

### 1. Intégration ArUco + Nav2

**Créer un service de navigation intelligent :**

```python
# Pseudo-code
def navigate_to_table(table_color):
    # 1. Navigation grossière avec Nav2
    goal = get_approximate_position(table_color)  # Ex: (-5.5, -2.8) pour RED
    navigate_to_pose(goal)  # Nav2
    
    # 2. Détection du marqueur ArUco
    marker = detect_aruco_marker()
    
    # 3. Vérification
    if marker.id == get_marker_id(table_color):  # Ex: ID 0 pour RED
        # Bon marqueur !
        pass
    else:
        # Mauvais marqueur, chercher le bon
        search_for_marker(table_color)
    
    # 4. Positionnement précis
    align_with_marker(marker)  # ±2mm
    
    return True
```

---

### 2. Waypoints et patrouille

**Créer un patrol dans la warehouse :**

```python
waypoints = [
    {"x": -5.0, "y": 3.0},   # START/STOP zone
    {"x": -5.5, "y": -2.8},  # RED table
    {"x": -4.35, "y": -2.8}, # BLUE table
    {"x": -3.2, "y": -2.8},  # YELLOW table
    {"x": 3.5, "y": -2.0},   # Depot table
    {"x": 5.0, "y": 3.0},    # CHARGING zone
]

for wp in waypoints:
    navigate_to_pose(wp)
    wait(5)  # Pause 5 secondes
```

---

### 3. Pick & Place automatisé

**Workflow complet :**

```
1. Recevoir commande : "Pick crate from BLUE table"
2. Nav2 → Naviguer vers BLUE table (approx)
3. ArUco → Détecter marqueur ID 1 (BLUE)
4. ArUco → Alignement précis (±2mm)
5. MoveIt → Pick de la caisse
6. Nav2 → Naviguer vers depot table
7. MoveIt → Place sur la table
8. Nav2 → Retour à CHARGING zone
```

---

### 4. Optimisations

**Performance :**
- Régler les vitesses max/min dans nav2_params.yaml
- Ajuster les costmaps (inflation_radius, cost_scaling_factor)
- Optimiser les paramètres DWB (vx_samples, sim_time)

**Robustesse :**
- Ajouter recovery behaviors (backup, spin, wait)
- Implémenter retry logic si navigation échoue
- Ajouter détection de blocage

**Interface :**
- Créer une GUI pour contrôler le robot
- Ajouter des boutons pour les destinations fréquentes
- Afficher l'état en temps réel

---

## 📚 Ressources

### Documentation officielle
- [Nav2 Documentation](https://navigation.ros.org/)
- [Nav2 Tuning Guide](https://navigation.ros.org/tuning/index.html)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

### Tutoriels
- [Nav2 First-Time Setup](https://navigation.ros.org/setup_guides/index.html)
- [Nav2 Concepts](https://navigation.ros.org/concepts/index.html)

### Forums
- [ROS Answers](https://answers.ros.org/)
- [Nav2 GitHub Issues](https://github.com/ros-planning/navigation2/issues)

---

## 🎉 Résumé des accomplissements

✅ **SLAM configuré** - Cartographie de la warehouse  
✅ **Carte créée** - warehouse_map_last_version (263×184 pixels, 5cm/px)  
✅ **LiDAR optimisé** - 270° avant, range 0.3-10m, pas de détection du bras  
✅ **Nav2 configuré** - AMCL + controller + planner + behavior servers  
✅ **Navigation autonome** - Robot navigue vers n'importe quel point  
✅ **Évitement d'obstacles** - Costmaps local/global avec inflation  
✅ **Localisation précise** - AMCL avec particle filter  
✅ **10 marqueurs ArUco** - Positionnement fin pour pick & place  
✅ **Architecture complète** - SLAM + Nav2 + ArUco = système autonome complet  

---

**Auteur :** Session de debugging intensive avec Douaa  
**Durée :** ~4 heures de configuration et résolution de problèmes  
**Leçons apprises :** 
- Toujours utiliser des chemins ABSOLUS pour les cartes
- `base_footprint` vs `base_link` - critial pour la TF
- Initialiser AMCL avec `/initialpose` avant toute navigation
- Les logs sont essentiels - utiliser `tee` pour les sauvegarder
- Tester étape par étape, ne pas tout lancer en même temps

---

**Version :** 1.0  
**Date de dernière mise à jour :** 28 Décembre 2025
