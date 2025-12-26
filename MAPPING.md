# 🗺️ Guide Complet - MAPPING du Warehouse

## 📋 Description

Ce guide explique comment créer une carte (map) de votre environnement warehouse en utilisant SLAM (Simultaneous Localization and Mapping) avec votre robot mobile manipulateur.

---

## 🎯 PLAN D'ACTION

### **ÉTAPE 1 : Ajouter un LiDAR au robot** (capteur pour scanner l'environnement)
### **ÉTAPE 2 : Rendre la base mobile contrôlable** (diff_drive_controller)
### **ÉTAPE 3 : Installer et configurer SLAM Toolbox**
### **ÉTAPE 4 : Téléopération pour explorer le warehouse**
### **ÉTAPE 5 : Sauvegarder la carte**
### **ÉTAPE 6 : Navigation autonome (optionnel)**

---

## 🚀 ÉTAPE 1 : AJOUTER UN LIDAR AU ROBOT

### Modifier le fichier URDF
```bash
nano ~/Lab_moveit/src/mobile_manipulator_sim/urdf/simple_mobile_robot.urdf
```

**Ajoutez AVANT la balise `</robot>` (après le plugin gazebo_ros2_control) :**
```xml
<!-- ========== LIDAR SENSOR ========== -->
<link name="lidar_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.05" length="0.06"/>
    </geometry>
    <material name="black">
      <color rgba="0.1 0.1 0.1 1"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.05" length="0.06"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.15 0 0.15" rpy="0 0 0"/>
</joint>

<!-- Plugin LiDAR Gazebo -->
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.2</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_laser" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
  <material>Gazebo/Black</material>
</gazebo>
```

---

## 🚗 ÉTAPE 2 : RENDRE LA BASE MOBILE CONTRÔLABLE

### A. Modifier les joints des roues
```bash
nano ~/Lab_moveit/src/mobile_manipulator_sim/urdf/simple_mobile_robot.urdf
```

**Cherchez les sections `<joint name="left_wheel_joint"` et `<joint name="right_wheel_joint"` et modifiez-les :**
```xml
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel"/>
  <origin xyz="0.15 0.225 -0.1" rpy="-1.57 0 0"/>
  <axis xyz="0 0 1"/>
  <dynamics damping="0.1" friction="0.1"/>
</joint>

<joint name="right_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="right_wheel"/>
  <origin xyz="0.15 -0.225 -0.1" rpy="-1.57 0 0"/>
  <axis xyz="0 0 1"/>
  <dynamics damping="0.1" friction="0.1"/>
</joint>
```

### B. Ajouter ros2_control pour les roues

**Dans la section `<ros2_control>`, ajoutez APRÈS les joints du bras :**
```xml
<!-- Roues pour navigation -->
<joint name="left_wheel_joint">
  <command_interface name="velocity"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>

<joint name="right_wheel_joint">
  <command_interface name="velocity"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```

### C. Ajouter le plugin diff_drive

**APRÈS le plugin gazebo_ros2_control, ajoutez :**
```xml
<!-- Plugin Differential Drive -->
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <update_rate>50</update_rate>
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.45</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>
    <max_wheel_torque>20</max_wheel_torque>
    <command_topic>cmd_vel</command_topic>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <publish_wheel_tf>false</publish_wheel_tf>
    <odometry_topic>odom</odometry_topic>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
  </plugin>
</gazebo>
```

---

## 📦 ÉTAPE 3 : INSTALLER SLAM TOOLBOX

### Installation des packages nécessaires
```bash
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-teleop-twist-keyboard
sudo apt install ros-humble-nav2-map-server
```

---

## 🔧 ÉTAPE 4 : CRÉER LE FICHIER DE CONFIGURATION SLAM

### Créer le répertoire config
```bash
mkdir -p ~/Lab_moveit/src/mobile_manipulator_sim/config
```

### Créer le fichier de paramètres SLAM
```bash
nano ~/Lab_moveit/src/mobile_manipulator_sim/config/mapper_params_online_async.yaml
```

**Collez ce contenu :**
```yaml
slam_toolbox:
  ros__parameters:
    # Plugin params
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping

    # Frequency
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 10.0
    minimum_time_interval: 0.5
    transform_publish_period: 0.02
    map_start_pose: [0.0, 0.0, 0.0]

    # General Parameters
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.2
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Correlation Parameters
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # Loop Closure Parameters
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # Scan Matcher Parameters
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

---

## 🚀 ÉTAPE 5 : CRÉER LE LAUNCH FILE POUR MAPPING

### Créer le fichier launch
```bash
nano ~/Lab_moveit/src/my_gazebo_world/launch/mapping.launch.py
```

**Collez ce contenu :**
```python
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    config_dir = get_package_share_directory('mobile_manipulator_sim')
    slam_params_file = os.path.join(config_dir, 'config', 'mapper_params_online_async.yaml')
    
    return LaunchDescription([
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file],
        ),
    ])
```

### Rendre le fichier exécutable
```bash
chmod +x ~/Lab_moveit/src/my_gazebo_world/launch/mapping.launch.py
```

---

## 📝 ÉTAPE 6 : METTRE À JOUR CMakeLists.txt

### Modifier CMakeLists.txt de my_gazebo_world
```bash
nano ~/Lab_moveit/src/my_gazebo_world/CMakeLists.txt
```

**Ajoutez (si pas déjà présent) :**
```cmake
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
```

### Modifier CMakeLists.txt de mobile_manipulator_sim
```bash
nano ~/Lab_moveit/src/mobile_manipulator_sim/CMakeLists.txt
```

**Vérifiez que cette section existe :**
```cmake
# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
```

---

## 🎮 ÉTAPE 7 : RECOMPILER LE PROJET

### Compilation complète
```bash
cd ~/Lab_moveit
colcon build --packages-select mobile_manipulator_sim my_gazebo_world
source install/setup.bash
```

---

## 🗺️ PROCÉDURE DE MAPPING COMPLÈTE

### Terminal 1 : Lancer Gazebo avec le robot
```bash
cd ~/Lab_moveit
source install/setup.bash
ros2 launch my_gazebo_world warehouse_with_robot.launch.py
```

⏱️ **Attendez que Gazebo charge complètement (~10-15 secondes)**

---

### Terminal 2 : Lancer SLAM Toolbox
```bash
source ~/Lab_moveit/install/setup.bash
ros2 launch my_gazebo_world mapping.launch.py
```

**✅ Vous devriez voir :**
```
[slam_toolbox]: Message Filter dropping message: frame 'laser'...
[slam_toolbox]: Registering sensor: [Custom Described Lidar]
```

---

### Terminal 3 : Téléopération (contrôle clavier)
```bash
source ~/Lab_moveit/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**🎮 Contrôles clavier :**

| Touche | Action |
|--------|--------|
| `i` | Avancer |
| `k` | Arrêter |
| `,` | Reculer |
| `j` | Tourner à gauche |
| `l` | Tourner à droite |
| `u` | Tourner gauche en avançant |
| `o` | Tourner droite en avançant |
| `m` | Tourner gauche en reculant |
| `.` | Tourner droite en reculant |
| `q` | Augmenter vitesse linéaire |
| `z` | Diminuer vitesse linéaire |
| `w` | Augmenter vitesse angulaire |
| `x` | Diminuer vitesse angulaire |
| `Espace` | Force l'arrêt |

**⚠️ Important :** Gardez le focus sur le terminal 3 pour que les commandes fonctionnent !

---

### Terminal 4 : Visualiser la carte dans RViz
```bash
source ~/Lab_moveit/install/setup.bash
rviz2
```

**Configuration RViz :**

1. **Fixed Frame** : Changez de `map` à `odom` puis à `map`
   - En haut à gauche : `Global Options` → `Fixed Frame` → `map`

2. **Ajouter la carte** :
   - Cliquez sur `Add` (en bas à gauche)
   - Sélectionnez `By topic`
   - Choisissez `/map` → `Map`
   - Cliquez `OK`

3. **Ajouter le LaserScan** :
   - Cliquez sur `Add`
   - Sélectionnez `By topic`
   - Choisissez `/scan` → `LaserScan`
   - Cliquez `OK`

4. **Ajouter le modèle du robot** :
   - Cliquez sur `Add`
   - Sélectionnez `By display type`
   - Choisissez `RobotModel`
   - Cliquez `OK`

5. **Ajouter la trajectoire (optionnel)** :
   - Cliquez sur `Add`
   - Sélectionnez `By display type`
   - Choisissez `Path`
   - Topic : `/trajectory`

**✅ Vous devriez voir :**
- Le robot au centre
- Les scans LiDAR en rouge/vert
- La carte se construire en gris/noir/blanc au fur et à mesure que vous explorez

---

## 🚶 ÉTAPE 8 : EXPLORER L'ENVIRONNEMENT

### Stratégie d'exploration

1. **Démarrez lentement** - Testez les contrôles
2. **Explorez méthodiquement** - Suivez les murs
3. **Couvrez toutes les zones** - N'oubliez pas les coins
4. **Fermez les boucles** - Revenez au point de départ pour améliorer la précision
5. **Évitez les mouvements brusques** - Le SLAM fonctionne mieux avec des mouvements fluides

### Conseils

- **Vitesse recommandée** : Linéaire = 0.2-0.3 m/s, Angulaire = 0.5-0.8 rad/s
- **Durée** : 5-10 minutes pour un warehouse complet
- **Vérifiez dans RViz** que la carte se construit correctement

---

## 💾 ÉTAPE 9 : SAUVEGARDER LA CARTE

### Une fois l'exploration terminée

**Dans un nouveau terminal (Terminal 5) :**
```bash
source ~/Lab_moveit/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/Lab_moveit/warehouse_map
```

**✅ Résultat :**

Deux fichiers sont créés dans `~/Lab_moveit/` :
- `warehouse_map.pgm` : Image de la carte (noir = obstacle, blanc = libre, gris = inconnu)
- `warehouse_map.yaml` : Métadonnées (résolution, origine, seuils)

### Vérifier les fichiers créés
```bash
ls -la ~/Lab_moveit/warehouse_map.*
```

**Vous devriez voir :**
```
-rw-rw-r-- 1 user user  XXXXX Dec 25 XX:XX warehouse_map.pgm
-rw-rw-r-- 1 user user    XXX Dec 25 XX:XX warehouse_map.yaml
```

---

## 📊 ÉTAPE 10 : VÉRIFIER LA QUALITÉ DE LA CARTE

### Visualiser la carte sauvegardée
```bash
eog ~/Lab_moveit/warehouse_map.pgm
```

OU
```bash
xdg-open ~/Lab_moveit/warehouse_map.pgm
```

### Critères de qualité

✅ **Bonne carte :**
- Murs bien définis (lignes noires nettes)
- Peu de zones grises (inconnues)
- Espaces ouverts en blanc
- Pas de "fantômes" ou doublons

❌ **Carte à refaire :**
- Murs flous ou dédoublés
- Beaucoup de zones grises
- Déformations importantes
- Obstacles mal positionnés

---

## 🔧 DÉPANNAGE

### Le LiDAR n'apparaît pas dans Gazebo
```bash
# Vérifier que le plugin est chargé
ros2 topic list | grep scan

# Devrait afficher: /scan
```

### Le robot ne bouge pas avec teleop
```bash
# Vérifier que cmd_vel est publié
ros2 topic echo /cmd_vel

# Appuyez sur 'i' dans le terminal teleop
# Vous devriez voir des messages
```

### SLAM ne démarre pas
```bash
# Vérifier les topics nécessaires
ros2 topic list | grep -E "scan|odom"

# Devrait afficher:
# /scan
# /odom
```

### La carte ne se construit pas dans RViz

1. Vérifiez que **Fixed Frame = map**
2. Vérifiez que le topic `/map` est actif : `ros2 topic hz /map`
3. Redémarrez SLAM Toolbox
4. Bougez le robot pour générer des scans

### Erreur "Transform timeout"
```bash
# Vérifier les frames
ros2 run tf2_tools view_frames

# Ouvrir le fichier généré
evince frames.pdf
```

---

## 📚 ÉTAPE 11 : UTILISER LA CARTE (NAVIGATION)

### Prochaines étapes

Une fois la carte créée, vous pouvez :

1. **Navigation autonome** avec Nav2
2. **Localisation** (AMCL) sur la carte existante
3. **Planification de trajectoires**
4. **Évitement d'obstacles dynamiques**

**📄 Voir le guide NAVIGATION.md pour la suite !**

---

## 📊 RÉSUMÉ : WORKFLOW MAPPING
```
┌─────────────────────────────────────────┐
│  1. Lancer Gazebo + Robot + LiDAR      │
└─────────────────┬───────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│  2. Lancer SLAM Toolbox                 │
└─────────────────┬───────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│  3. Lancer RViz (visualisation)         │
└─────────────────┬───────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│  4. Téléopérer le robot (exploration)   │
└─────────────────┬───────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│  5. Sauvegarder la carte                │
└─────────────────────────────────────────┘
```

---

## 🎓 CONCEPTS UTILISÉS

| Concept | Description |
|---------|-------------|
| **SLAM** | Simultaneous Localization And Mapping |
| **LiDAR** | Light Detection And Ranging (capteur laser) |
| **Odométrie** | Estimation de position basée sur les roues |
| **tf2** | Système de transformations ROS2 |
| **Occupancy Grid** | Grille d'occupation (carte 2D) |
| **Loop Closure** | Détection de retour au point de départ |
| **Differential Drive** | Locomotion différentielle (2 roues) |

---

## 📁 FICHIERS CRÉÉS

| Fichier | Chemin | Description |
|---------|--------|-------------|
| **Carte PGM** | `~/Lab_moveit/warehouse_map.pgm` | Image de la carte |
| **Métadonnées** | `~/Lab_moveit/warehouse_map.yaml` | Configuration carte |
| **Config SLAM** | `~/Lab_moveit/src/mobile_manipulator_sim/config/mapper_params_online_async.yaml` | Paramètres SLAM |
| **Launch mapping** | `~/Lab_moveit/src/my_gazebo_world/launch/mapping.launch.py` | Fichier de lancement |

---

## 🎉 FÉLICITATIONS !

Vous savez maintenant :
- ✅ Ajouter un LiDAR à votre robot
- ✅ Rendre la base mobile contrôlable
- ✅ Configurer SLAM Toolbox
- ✅ Téléopérer le robot
- ✅ Créer une carte de l'environnement
- ✅ Sauvegarder et visualiser la carte

**Prochaine étape : Navigation autonome ! 🚀**

---

## 📚 RESSOURCES

- [SLAM Toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
- [ROS2 Navigation](https://navigation.ros.org/)
- [Gazebo Sensors](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#Laser)
- [Differential Drive Plugin](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#DifferentialDrive)

---

**🗺️ Bon mapping ! 🤖✨**
