# Lab MoveIt - UR5 & Panda Tutorials

Dépôt pour les travaux pratiques de robotique avec MoveIt2 et ROS2 Humble.

## 📦 Packages inclus

### Packages UR5
- **ur_yt_sim** : Simulation UR5 avec Gazebo et pick & place
- **ur5_camera_gripper_moveit_config** : Configuration MoveIt pour UR5
- **IFRA_LinkAttacher** : Plugin pour attacher/détacher des objets

### Packages Panda
- **my_panda_moveit_config** : Configuration MoveIt avec poses personnalisées
- **my_panda_poses** : Scripts Python pour contrôle automatique des poses

## 🚀 Installation

### Prérequis
```bash
sudo apt update
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-resources-panda-moveit-config
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
```

### Setup du workspace
```bash
# Cloner le dépôt
cd ~
git clone https://github.com/faraah16/Lab_moveit.git

# Créer le workspace ROS2
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
cp -r ~/Lab_moveit/src/* .

# Installer les dépendances
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y

# Compiler
colcon build
source install/setup.bash
```

## 🎯 Utilisation

### UR5 - Pick & Place
```bash
cd ~/robot_ws
source install/setup.bash
ros2 launch ur_yt_sim <votre_launch>.launch.py
```

### Panda - Contrôle de poses

**Terminal 1** - Lancer MoveIt :
```bash
cd ~/robot_ws
source install/setup.bash
ros2 launch my_panda_moveit_config demo.launch.py
```

**Terminal 2** - Contrôler avec Python :
```bash
cd ~/robot_ws/src/my_panda_poses/scripts
python3 panda_move_to_pose.py
```

**Commandes disponibles** :
- `1` : my_home (position repos)
- `2` : my_ready (position prête)
- `3` : my_pick (position saisie)
- `4` : my_extended (bras étendu)
- `6` : Séquence automatique
- `q` : Quitter

### Panda - Tester dans RViz
Dans RViz → Panneau MotionPlanning → "Select Goal State" → Choisir une pose → "Plan & Execute"

## 📝 Ajouter une pose avec MoveIt Setup Assistant
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

1. "Edit Existing MoveIt Configuration Package"
2. Charger `~/robot_ws/src/my_panda_moveit_config`
3. Aller dans "Robot Poses"
4. "Add Pose" → Définir les valeurs → "Save"
5. "Configuration Files" → "Generate Package"
6. Recompiler : `colcon build --packages-select my_panda_moveit_config`

## 🔧 Structure
```
Lab_moveit/
└── src/
    ├── IFRA_LinkAttacher/
    ├── ur5_camera_gripper_moveit_config/
    ├── ur_yt_sim/
    ├── my_panda_poses/
    └── my_panda_moveit_config/
```

## 🐛 Dépannage

**Package not found** :
```bash
source ~/robot_ws/install/setup.bash
```

**Erreur de contrôleurs Panda** :
Vérifier que `ros2_controllers.yaml` a les `command_interfaces: [position]` et `state_interfaces: [position, velocity]`.

## 👥 Auteur
Farah - Instructeur

## 📄 Licence
Educational use
