# 🤖 Projet Robot Mobile Manipulateur - Guide d'Utilisation

## 📋 Description du Projet

Robot mobile avec bras manipulateur 2-DOF et gripper fonctionnel dans l'environnement Gazebo.

**Caractéristiques :**
- Base mobile bleue avec roues
- Bras robotique à 2 articulations (joint1 + joint2)
- Gripper à 2 doigts prismatiques
- Contrôle via ROS2 Humble
- Environnement : Warehouse Gazebo

---

## 🔧 1. COMPILATION DU PROJET
```bash
cd ~/Lab_moveit
colcon build --packages-select mobile_manipulator_sim my_gazebo_world
source install/setup.bash
```

**ℹ️ Note :** Recompilez après toute modification des fichiers URDF ou de configuration.

---

## 🌍 2. LANCER LE WORLD AVEC LE ROBOT

### Terminal 1 - Lancement Gazebo
```bash
cd ~/Lab_moveit
source install/setup.bash
ros2 launch my_gazebo_world warehouse_with_robot.launch.py
```

⏱️ **Attendez ~10-15 secondes** que Gazebo charge complètement avant de passer à l'étape suivante.

---

## 🎮 3. CHARGER ET ACTIVER LES CONTROLLERS

### Terminal 2 - Configuration Controllers (une seule fois après le lancement)
```bash
source ~/Lab_moveit/install/setup.bash

# Charger les controllers
ros2 control load_controller joint_state_broadcaster
ros2 control load_controller arm_controller
ros2 control load_controller gripper_controller

# Activer les controllers
ros2 control set_controller_state joint_state_broadcaster start
ros2 control set_controller_state arm_controller start
ros2 control set_controller_state gripper_controller start
```

### Vérifier que les controllers sont actifs
```bash
ros2 control list_controllers
```

**✅ Résultat attendu :**
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
arm_controller[joint_trajectory_controller/JointTrajectoryController] active
gripper_controller[position_controllers/JointGroupPositionController] active
```

---

## 🎬 4. LANCER LA DÉMONSTRATION AUTOMATIQUE

### Terminal 2 ou 3 - Script de démonstration
```bash
source ~/Lab_moveit/install/setup.bash
python3 ~/Lab_moveit/demo_robot.py
```

**🎥 La démonstration exécute automatiquement :**
1. Position HOME
2. Rotation de la base (joint1)
3. Mouvement du coude (joint2)
4. Test d'ouverture/fermeture du gripper
5. Séquence pick & place simulée
6. Retour à la position HOME

---

## 🎮 5. CONTRÔLE MANUEL DU ROBOT

### Commandes de base

#### Bouger le bras (exemple : rotation base + coude)
```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1', 'joint2'],
  points: [{positions: [1.57, 0.5], time_from_start: {sec: 3}}]
}"
```

**Valeurs des joints :**
- `joint1` : -3.14 à 3.14 (rotation base)
- `joint2` : -1.57 à 1.57 (coude)

#### Contrôle du gripper

**Ouvrir le gripper :**
```bash
ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.04, -0.04]}"
```

**Fermer le gripper :**
```bash
ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"
```

**Position intermédiaire :**
```bash
ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.02, -0.02]}"
```

---

## ✅ 6. COMMANDES DE VÉRIFICATION

### Vérifier les controllers actifs
```bash
ros2 control list_controllers
```

### Voir tous les topics ROS2
```bash
ros2 topic list
```

### Voir l'état des joints en temps réel
```bash
ros2 topic echo /joint_states
```

### Lister les services disponibles
```bash
ros2 service list
```

### Vérifier les interfaces du controller_manager
```bash
ros2 control list_hardware_interfaces
```

---

## 🔄 7. WORKFLOW COMPLET (Commande unique)

Pour les utilisateurs avancés, voici une séquence complète :
```bash
# Terminal 1 - Lancer Gazebo
cd ~/Lab_moveit && source install/setup.bash
ros2 launch my_gazebo_world warehouse_with_robot.launch.py
```
```bash
# Terminal 2 - Charger controllers + Lancer démo
source ~/Lab_moveit/install/setup.bash

# Attendre 15 secondes après le lancement de Gazebo, puis :
ros2 control load_controller joint_state_broadcaster && \
ros2 control load_controller arm_controller && \
ros2 control load_controller gripper_controller && \
ros2 control set_controller_state joint_state_broadcaster start && \
ros2 control set_controller_state arm_controller start && \
ros2 control set_controller_state gripper_controller start

# Attendre 2 secondes, puis lancer la démo :
python3 ~/Lab_moveit/demo_robot.py
```

---

## 📁 8. STRUCTURE DU PROJET
```
~/Lab_moveit/
├── src/
│   ├── mobile_manipulator_sim/
│   │   ├── urdf/
│   │   │   └── simple_mobile_robot.urdf       # Description du robot
│   │   ├── config/
│   │   │   └── robot_controllers.yaml         # Configuration controllers
│   │   └── CMakeLists.txt
│   └── my_gazebo_world/
│       ├── worlds/
│       │   └── warehouse.world                # Monde Gazebo
│       └── launch/
│           └── warehouse_with_robot.launch.py # Fichier de lancement
├── demo_robot.py                              # Script de démonstration
└── README.md                                  # Ce fichier
```

---

## 📝 9. FICHIERS IMPORTANTS

| Fichier | Chemin | Description |
|---------|--------|-------------|
| **Robot URDF** | `~/Lab_moveit/src/mobile_manipulator_sim/urdf/simple_mobile_robot.urdf` | Modèle 3D et physique du robot |
| **Controllers** | `~/Lab_moveit/src/mobile_manipulator_sim/config/robot_controllers.yaml` | Configuration ros2_control |
| **Launch file** | `~/Lab_moveit/src/my_gazebo_world/launch/warehouse_with_robot.launch.py` | Lancement Gazebo + Robot |
| **Script démo** | `~/Lab_moveit/demo_robot.py` | Programme de démonstration |
| **World** | `~/Lab_moveit/src/my_gazebo_world/worlds/warehouse.world` | Environnement Gazebo |

---

## 🛠️ 10. DÉPANNAGE

### Le robot ne spawn pas
```bash
# Vérifier que le package est compilé
ros2 pkg list | grep mobile_manipulator_sim

# Recompiler si nécessaire
cd ~/Lab_moveit
rm -rf build install
colcon build
source install/setup.bash
```

### Les controllers ne se chargent pas
```bash
# Vérifier que gazebo_ros2_control est chargé
ros2 service list | grep controller_manager

# Si aucun service, vérifier les logs Gazebo pour des erreurs
```

### Le robot tombe ou se comporte bizarrement

- Vérifier que la simulation est en **temps réel** (Real Time Factor proche de 1.0)
- Mettre en pause Gazebo, repositionner le robot manuellement
- Relancer la simulation

### Erreur "Could not contact service"

- Attendre plus longtemps après le lancement de Gazebo (15-20 secondes)
- Vérifier que Gazebo est complètement chargé avant de charger les controllers

---

## 🎓 11. CONCEPTS ROS2 UTILISÉS

- **URDF** : Unified Robot Description Format
- **ros2_control** : Framework de contrôle temps réel
- **Gazebo** : Simulateur robotique
- **JointTrajectoryController** : Contrôle de trajectoire
- **JointGroupPositionController** : Contrôle de position
- **joint_state_broadcaster** : Publication d'état des joints

---

## 📚 12. RESSOURCES SUPPLÉMENTAIRES

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Classic Documentation](https://classic.gazebosim.org/)
- [ros2_control Documentation](https://control.ros.org/master/index.html)
- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)

---

## 👤 INFORMATIONS

**Projet :** Robot Mobile Manipulateur  
**ROS Version :** ROS2 Humble  
**Gazebo Version :** Gazebo Classic 11  
**Système :** Ubuntu 22.04  
**Date :** Décembre 2025  

---

## 🎉 FÉLICITATIONS !

Vous avez maintenant un robot mobile manipulateur fonctionnel avec :
- ✅ Contrôle du bras robotique
- ✅ Gripper fonctionnel
- ✅ Démonstration automatique
- ✅ Contrôle manuel via ROS2

**Bon codage robotique ! 🤖✨**
