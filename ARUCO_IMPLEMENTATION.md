# 🎯 Implémentation de la Détection ArUco - Session du 27 Décembre 2024

## 📋 Objectif
Ajouter une caméra au robot mobile et implémenter la détection de markers ArUco pour permettre un positionnement précis dans les zones colorées (rouge, bleue, verte/jaune).

---

## ✅ Accomplissements

### 1. **Ajout d'une Caméra RGB au Robot**
- **Fichier modifié** : `~/Lab_moveit/src/mobile_manipulator_sim/urdf/simple_mobile_robot.urdf`
- **Position** : Sous le châssis (z=0.05m), orientée vers le sol
- **Orientation** : rpy="0 1.5708 0" (90° vers le bas)
- **FOV** : 1.5708 radians (90°) pour large champ de vision
- **Résolution** : 640x480 pixels
- **Topic de publication** : `/rgb_camera/image_raw`

### 2. **Génération des Markers ArUco**
- **Dictionnaire utilisé** : DICT_4X4_50
- **Taille physique** : 20cm × 20cm (0.20m)
- **Markers créés** :
  - ID 0 → Zone ROUGE
  - ID 1 → Zone BLEUE
  - ID 2 → Zone VERTE/JAUNE
- **Fichiers** : 
  - `marker_red_table.png`
  - `marker_blue_table.png`
  - `marker_yellow_table.png`
- **Emplacement** : `~/Lab_moveit/src/my_gazebo_world/markers/`

### 3. **Placement des Markers dans Gazebo**
- **Fichier modifié** : `~/Lab_moveit/src/my_gazebo_world/worlds/warehouse.world`
- **Position** : Au sol (z=0.025m) dans chaque zone colorée
- **Positions exactes** :
  - Marker Rouge : (-5.5, -2.8, 0.025)
  - Marker Bleu : (-4.35, -2.8, 0.025)
  - Marker Jaune : (-3.2, -2.8, 0.025)
- **Matériaux** : Configurés dans `~/Lab_moveit/src/my_gazebo_world/materials/`

### 4. **Nœud de Détection ArUco Personnalisé**
- **Fichier créé** : `~/Lab_moveit/aruco_detector_node.py`
- **Fonctionnalités** :
  - Détection en temps réel des 3 markers (ID 0, 1, 2)
  - Publication de l'image avec détections visuelles (carrés verts + IDs)
  - Logs de détection en continu
- **Topics publiés** :
  - `/aruco_detection/result` (Image avec détections dessinées)

---

## 🔧 Fichiers Créés/Modifiés

### Fichiers Modifiés
1. `~/Lab_moveit/src/mobile_manipulator_sim/urdf/simple_mobile_robot.urdf`
   - Ajout de `camera_link` et `camera_joint`
   - Plugin Gazebo `libgazebo_ros_camera.so`

2. `~/Lab_moveit/src/my_gazebo_world/worlds/warehouse.world`
   - Ajout de 3 models ArUco (aruco_marker_red, blue, yellow)

3. `~/Lab_moveit/src/my_gazebo_world/launch/warehouse_with_robot.launch.py`
   - Ajout des nœuds aruco_ros (non fonctionnels, remplacés par nœud custom)

### Fichiers Créés
1. `~/Lab_moveit/aruco_detector_node.py` ⭐ (PRINCIPAL)
2. `~/Lab_moveit/generate_aruco_markers.py` (script de génération)
3. `~/Lab_moveit/test_aruco_detection_v2.py` (script de test)
4. `~/Lab_moveit/src/my_gazebo_world/markers/*.png` (3 images)
5. `~/Lab_moveit/src/my_gazebo_world/materials/scripts/aruco_markers.material`
6. `~/Lab_moveit/src/my_gazebo_world/materials/textures/*.png` (3 textures)

---

## 🚀 Commandes de Test

### **TERMINAL 1 - Lancer Gazebo + Robot**
```bash
cd ~/Lab_moveit
source install/setup.bash
ros2 launch my_gazebo_world warehouse_with_robot.launch.py
```
**Attendez que Gazebo soit complètement chargé**

---

### **TERMINAL 2 - Lancer le Détecteur ArUco**
```bash
cd ~/Lab_moveit
source install/setup.bash
python3 ~/Lab_moveit/aruco_detector_node.py
```
**Vous devriez voir :**
- `=== Custom ArUco Detector DEMARRE ===`
- `Frame 30 - Noeud actif` (toutes les secondes)
- `>>> DETECTION: Markers [X] <<<` (quand un marker est visible)

---

### **TERMINAL 3 - Visualiser la Détection**
```bash
cd ~/Lab_moveit
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```
**Sélectionnez le topic** : `/aruco_detection/result`

**Vous devriez voir :**
- Les markers avec **carrés verts** autour
- Les **IDs affichés** (0, 1, ou 2)

---

### **TERMINAL 4 - Contrôler le Robot**
```bash
cd ~/Lab_moveit
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
**Commandes :**
- `i` : Avancer
- `k` : Reculer
- `j` : Tourner à gauche
- `l` : Tourner à droite
- `q/z` : Augmenter/diminuer vitesse

---

### **Vérifications (optionnel)**
```bash
# Vérifier que le nœud publie
ros2 topic hz /aruco_detection/result

# Voir la liste des markers détectés
ros2 topic echo /aruco_detection/result --once

# Lister tous les topics caméra
ros2 topic list | grep camera

# Lister tous les topics ArUco
ros2 topic list | grep aruco
```

---

## 🐛 Problèmes Rencontrés et Solutions

### **Problème 1 : aruco_ros ne détectait rien**
**Cause** : Le package aruco_ros utilise un dictionnaire différent de DICT_4X4_50 et le paramètre `dictionary` n'est pas déclaré.

**Solution** : Création d'un nœud personnalisé Python qui utilise explicitement DICT_4X4_50.

---

### **Problème 2 : Caméra voit les tables mais pas les markers complets**
**Cause** : 
- Caméra trop proche du sol (17cm)
- FOV trop étroit (60°)

**Solution** :
- Monter la caméra à z=0.05 (25cm du sol)
- Augmenter FOV à 90° (1.5708 rad)

---

### **Problème 3 : /camera/image_raw noir, /rgb_camera/image_raw fonctionnel**
**Cause** : Deux caméras publient sur des topics différents, le nœud écoutait le mauvais.

**Solution** : Configurer le nœud custom pour écouter `/rgb_camera/image_raw`.

---

### **Problème 4 : Markers sous les tables (invisibles)**
**Cause** : Position y=-3.5 directement sous les tables.

**Solution** : Déplacer les markers à y=-2.8 (70cm vers l'avant).

---

## 📊 Spécifications Techniques

### Caméra
- **Type** : RGB
- **Position** : (0, 0, 0.05) relative à base_link
- **Orientation** : Pitch = 90° (vers le bas)
- **FOV** : 90° horizontal
- **Résolution** : 640×480
- **Framerate** : 30 Hz
- **Topic** : `/rgb_camera/image_raw`

### Markers ArUco
- **Dictionnaire** : DICT_4X4_50
- **Taille** : 0.20m × 0.20m (20cm)
- **Résolution image** : 300×300 pixels
- **Format** : PNG noir et blanc

### Détection
- **Nœud** : custom_aruco_detector
- **Algorithme** : OpenCV ArUco (cv2.aruco.detectMarkers)
- **Performance** : ~30 FPS
- **Output** : Image annotée + logs

---

## 🎯 Prochaines Étapes

### Court Terme
1. ⏳ Intégrer le nœud Python dans le launch file (démarrage automatique) --------->DONE✅
2. ⏳ Implémenter la navigation autonome vers un marker spécifique
3. ⏳ Créer un service ROS2 : "navigue vers marker ID X"

### Moyen Terme
1. ⏳ Calculer la pose 3D précise des markers (position + orientation)
2. ⏳ Publier les TF transforms pour chaque marker détecté
3. ⏳ Intégrer avec Nav2 pour navigation autonome
4. ⏳ Implémenter pick & place basé sur les markers

### Long Terme
1. ⏳ Détection de la couleur des boîtes (vision par ordinateur)
2. ⏳ Séquence complète : Départ → Navigation marker → Pick → Retour
3. ⏳ Interface utilisateur pour commander le robot

---

## 📝 Notes Importantes

### Performance
- Sur machines peu puissantes : possible lenteur/lag dans la détection
- Solution : Réduire la fréquence de détection (skip frames)

### Limitation Actuelle
- Le nœud détecte mais ne publie pas encore les poses 3D
- Nécessaire pour navigation précise vers le marker

### Backup
- Code sauvegardé sur GitHub : `git push origin main`
- Fichiers importants dans `/mnt/user-data/outputs/`

---

## 🏆 Résultat Final

✅ **Robot équipé d'une caméra fonctionnelle**  
✅ **3 markers ArUco placés dans l'environnement**  
✅ **Détection en temps réel des 3 markers**  
✅ **Visualisation des détections (carrés verts + IDs)**  
✅ **Base solide pour la navigation autonome**  

**Prêt pour la navigation vers les markers et le pick & place ! 🚀**

---

**Date** : 27 Décembre 2024  
**Auteur** : Douaa  
**Projet** : Robot Mobile Manipulateur - Préparation Commandes Autonome  
**Institution** : Université Euromed de Fès (UEMF)
