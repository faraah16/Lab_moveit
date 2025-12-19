# 🛠️ Lab_moveit – Guide de Build & Débogage (ROS 2 Humble)

Ce document explique **pas à pas** comment cloner, corriger et builder correctement le projet **Lab_moveit**
(ROS 2 Humble + MoveIt + Gazebo + IFRA LinkAttacher).

Il est destiné à toute personne qui clone le projet pour la première fois.

---

## 📦 Prérequis

- OS : **Ubuntu 22.04**
- ROS : **ROS 2 Humble**

```bash
sudo apt update
sudo apt install ros-humble-desktop -y
sudo apt install ros-humble-moveit -y
```

Toujours sourcer ROS :
```bash
source /opt/ros/humble/setup.bash
```

---

## 📥 Clonage du projet

```bash
cd ~
git clone https://github.com/faraah16/Lab_moveit.git
cd Lab_moveit
```

⚠️ **Important**
- Ne jamais lancer `colcon build` depuis `~`
- Toujours builder depuis le dossier du workspace (`Lab_moveit`)

---

## ❗ Problèmes courants

Erreurs possibles :
- `Duplicate package names not supported`
- `Could not find linkattacher_msgsConfig.cmake`
- `Could not find ros2_linkattacherConfig.cmake`

Causes :
- services ROS2 mal exportés
- ordre de build incorrect
- dépendances CMake invisibles

---

## ✅ Correction — linkattacher_msgs

📁 `Lab_moveit/src/IFRA_LinkAttacher/linkattacher_msgs/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.5)
project(linkattacher_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AttachLink.srv"
  "srv/DetachLink.srv"
)

ament_export_dependencies(rosidl_default_runtime)

install(
  DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

⚠️ Sans `ament_export_dependencies(rosidl_default_runtime)`,
le fichier `linkattacher_msgsConfig.cmake` n’est pas généré.

---

## 🔗 Ordre des dépendances

```
linkattacher_msgs
        ↓
ros2_linkattacher
        ↓
ur_yt_sim
```

---

## 🧹 Nettoyage

```bash
cd ~/Lab_moveit
rm -rf build install log
```

---

## 🔨 Build (ORDRE OBLIGATOIRE)

### 1️⃣ Services
```bash
colcon build --packages-select linkattacher_msgs
source install/setup.bash
```

### 2️⃣ Serveur LinkAttacher
```bash
colcon build --packages-select ros2_linkattacher
source install/setup.bash
```

### 3️⃣ Simulation
```bash
colcon build --packages-select ur_yt_sim
```

### 4️⃣ Workspace complet
```bash
colcon build
```

---

## 🧪 Vérification

```bash
source install/setup.bash
ros2 interface list | grep linkattacher
ros2 service list | grep attach
```

Résultat attendu :
```
linkattacher_msgs/srv/AttachLink
linkattacher_msgs/srv/DetachLink
/attach_link
/detach_link
```

---

## 🚀 Lancement

```bash
ros2 launch ur_yt_sim <launch_file>.launch.py
```

---

## 🧠 Bonnes pratiques

- Toujours builder les `.msg` / `.srv` en premier
- Toujours `source install/setup.bash`
- CMake est **case-sensitive**
- `ament_package()` et `ament_export_dependencies()` sont obligatoires

---

## 🎯 Conclusion

Ce guide permet de cloner et builder **Lab_moveit** sans erreur
sur **ROS 2 Humble**, avec MoveIt, Gazebo et IFRA LinkAttacher.
