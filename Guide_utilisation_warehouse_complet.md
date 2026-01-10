# 📚 **GUIDE COMPLET D'UTILISATION DU PROJET WAREHOUSE**

---

## 🚀 **PARTIE 1 : DÉMARRAGE DU SYSTÈME**

### **Étape 1 : Lancer Gazebo + Robot + Battery Manager**

```bash
# Terminal 1
cd ~/Lab_moveit
source install/setup.bash
ros2 launch my_gazebo_world warehouse_with_robot.launch.py
```

**⏳ ATTENDRE 60 SECONDES** que Gazebo charge complètement

**✅ Vous devez voir :**
- Fenêtre Gazebo avec le warehouse
- Robot à la position start/stop zone (-5.0, 3.1)
- Lidar (cylindre bleu) qui tourne
- Logs : `[BATTERY] 🔋 Battery Manager started: 100%`

---

### **Étape 2 : Lancer Navigation + Warehouse System**

```bash
# Terminal 2 (NOUVEAU terminal)
cd ~/Lab_moveit
source install/setup.bash
ros2 launch my_gazebo_world warehouse_system.launch.py
```

**⏳ ATTENDRE 15 SECONDES**

**✅ Vous devez voir :**
```
[ORCHESTR] 🎯 Mission Orchestrator démarré
[STOCK] 📦 Stock Manager démarré
[QUEUE] ⚡ Mission Queue Manager démarré

╔════════════════════════════════════════════════════════════╗
║  ✅ WAREHOUSE SYSTEM PRÊT !                               ║
╚════════════════════════════════════════════════════════════╝
```

---

### **Étape 3 : Lancer l'Interface Employé**

```bash
# Terminal 3 (NOUVEAU terminal)
cd ~/Lab_moveit
source install/setup.bash
ros2 run warehouse_manager employee_interface
```

**✅ Vous devez voir l'interface CLI :**

```
╔═══════════════════════════════════════════════════════════╗
║          🏭 WAREHOUSE CONTROL SYSTEM 🏭                   ║
╚═══════════════════════════════════════════════════════════╝

┌───────────────────────────────────────────────────────────┐
│  ÉTAT DU SYSTÈME                                          │
├───────────────────────────────────────────────────────────┤
│  🟢 Batterie:  100.0% [██████████]                        │
│  📍 Position: start_stop_zone                             │
│  🎯 Mission:  Aucune                                      │
└───────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────┐
│  📦 STOCK ACTUEL                                          │
├───────────────────────────────────────────────────────────┤
│  🔴 Red Boxes   :  5 total  (Depot:  2 | Table:  3)      │
│  🔵 Blue Boxes  :  3 total  (Depot:  1 | Table:  2)      │
│  🟡 Yellow Boxes:  2 total  (Depot:  0 | Table:  2)      │
└───────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────┐
│  COMMANDES DISPONIBLES                                    │
├───────────────────────────────────────────────────────────┤
│  [1] 📝 Modifier stock initial                            │
│  [2] 📦 Nouveau arrivage                                  │
│  [3] 🎯 Créer commande client                             │
│  [4] 🛑 Arrêter robot                                     │
│  [5] 📊 Afficher historique                               │
│  [6] 🔄 Rafraîchir affichage                              │
│  [0] ❌ Quitter                                            │
└───────────────────────────────────────────────────────────┘

Votre choix: _
```

---

## 📋 **PARTIE 2 : SCÉNARIOS D'UTILISATION**

---

### **🎯 SCÉNARIO 1 : MODIFIER LE STOCK INITIAL**

**Quand l'utiliser :** Au début de la journée pour définir l'état du warehouse

**Étapes :**

1. **Dans Terminal 3** (Interface), tape : `1`

2. **Le système affiche le stock actuel :**
```
📦 STOCK ACTUEL:
  🔴 Red Boxes   :  5 total  (Depot:  2 | Table:  3)
  🔵 Blue Boxes  :  3 total  (Depot:  1 | Table:  2)
  🟡 Yellow Boxes:  2 total  (Depot:  0 | Table:  2)

Voulez-vous modifier ? (o/n):
```

3. **Tape :** `o`

4. **Entre les nouvelles quantités :**
```
🔴 Red Boxes:
  Nombre dans depot_table: 5
  Nombre dans red_table: 10

🔵 Blue Boxes:
  Nombre dans depot_table: 3
  Nombre dans blue_table: 7

🟡 Yellow Boxes:
  Nombre dans depot_table: 2
  Nombre dans yellow_table: 4
```

5. **Confirme :** `o`

**✅ Résultat :**
```
✅ Stock mis à jour avec succès !
💾 Stock sauvegardé dans YAML
```

**📊 Dans Terminal 2, tu verras :**
```
[STOCK] 📝 Réception commande: Mise à jour stock
[STOCK] 💾 Stock sauvegardé dans YAML
```

---

### **📦 SCÉNARIO 2 : ENREGISTRER UN NOUVEL ARRIVAGE**

**Quand l'utiliser :** Quand des boxes arrivent au dépôt

**Étapes :**

1. **Dans Terminal 3**, tape : `2`

2. **Entre les quantités par couleur :**
```
📦 NOUVEL ARRIVAGE

Combien de boxes sont arrivées ?

🔴 Red Boxes     : 3
🔵 Blue Boxes    : 5
🟡 Yellow Boxes  : 2
```

3. **Le système affiche un récapitulatif :**
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📋 RÉCAPITULATIF
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  🔴 Red: 3 box(es)
  🔵 Blue: 5 box(es)
  🟡 Yellow: 2 box(es)
  
  📊 Total: 10 box(es)

Confirmer l'arrivage ? (o/n):
```

4. **Confirme :** `o`

**✅ Résultat :**
```
✅ Arrivage enregistré !
🤖 Le robot va trier les boxes automatiquement...
```

**📊 Dans Terminal 2 (Queue Manager) :**
```
[QUEUE] 📦 Mission ajoutée: Tri arrivage (10 boxes)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📋 FILE DES MISSIONS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  1. 🟢 Tri arrivage (10 boxes)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🚀 EXÉCUTION PROCHAINE MISSION
   Tri arrivage (10 boxes)
```

**🤖 Dans Terminal 2 (Orchestrator) :**
```
[ORCHESTR] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[ORCHESTR] 📦 MISSION REÇUE: Tri arrivage (10 boxes)
[ORCHESTR] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[ORCHESTR] 
[ORCHESTR] 📦 MISSION: Tri d'arrivage
[ORCHESTR]    → Navigation vers depot_table
[ORCHESTR] 
[ORCHESTR] 🎯 MISSION: Aller à depot_table
[ORCHESTR] 🚀 PHASE 1: Rotation vers la cible
[ORCHESTR] 🧭 Angle à tourner: X.XX rad
[ORCHESTR] ⏳ Rotation en cours... (Xms écoulées)
[ORCHESTR] ✅ Rotation terminée !
[ORCHESTR] 
[ORCHESTR] 🚀 PHASE 2: Navigation vers la position
[ORCHESTR] ✅ Arrivé au depot_table
[ORCHESTR] 🔄 Tri des boxes en cours...
[ORCHESTR] ✅ Tri de 10 boxes terminé (simulé)
[ORCHESTR] 💡 Les boxes sont maintenant sur leurs tables
[ORCHESTR] 
[ORCHESTR] ✅ MISSION TERMINÉE AVEC SUCCÈS
```

**🎮 Dans Gazebo :**
- Robot se déplace vers depot_table
- Robot s'arrête à depot_table
- Attend 5 secondes (simulation tri)
- Mission terminée

---

### **🎯 SCÉNARIO 3 : CRÉER UNE COMMANDE CLIENT**

**Quand l'utiliser :** Un client passe une commande

**Étapes :**

1. **Dans Terminal 3**, tape : `3`

2. **Entre le nom du client :**
```
🎯 NOUVELLE COMMANDE CLIENT

👤 Nom du client: Restaurant Le Gourmet
```

3. **Entre les quantités demandées :**
```
📦 Items commandés:

🔴 Red Boxes     (dispo: 10): 2
🔵 Blue Boxes    (dispo: 7): 1
🟡 Yellow Boxes  (dispo: 4): 0
```

4. **Choisis la destination :**
```
📍 DESTINATION:
  [1] Yellow Crate Left
  [2] Yellow Crate Right

Votre choix: 1
```

5. **Choisis la priorité :**
```
⚡ PRIORITÉ:
  [1] 🔴 Haute (urgent)
  [2] 🟡 Normale (standard)
  [3] 🟢 Basse (peut attendre)

Votre choix: 1
```

6. **Récapitulatif affiché :**
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📋 RÉCAPITULATIF DE LA COMMANDE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  📋 Numéro: CMD001
  👤 Client: Restaurant Le Gourmet
  
  📦 Items:
     🔴 2x Red Boxes
     🔵 1x Blue Boxes
  
  📍 Destination: Yellow Crate Left
  ⚡ Priorité: 🔴 Haute

Confirmer la commande ? (o/n):
```

7. **Confirme :** `o`

**✅ Résultat :**
```
✅ Commande enregistrée !
🤖 Le robot va préparer la commande...
```

**📊 Dans Terminal 2 (Queue Manager) :**
```
[QUEUE] 🔴 Mission ajoutée: Commande CMD001 - Restaurant Le Gourmet

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📋 FILE DES MISSIONS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  1. 🔴 Commande CMD001 - Restaurant Le Gourmet
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

**🤖 Dans Terminal 2 (Orchestrator) :**
```
[ORCHESTR] 🎯 MISSION: Commande CMD001
[ORCHESTR]    Destination: yellow_crate_left
[ORCHESTR] 
[ORCHESTR] 📦 Récupération: 2x Red
[ORCHESTR]    → Navigation vers red_table
[ORCHESTR] ✅ Arrivé à red_table
[ORCHESTR] 🦾 Pick de 2 box(es) en cours...
[ORCHESTR] ✅ 2 box(es) récupérée(s)
[ORCHESTR] 
[ORCHESTR] 📦 Récupération: 1x Blue
[ORCHESTR]    → Navigation vers blue_table
[ORCHESTR] ✅ Arrivé à blue_table
[ORCHESTR] 🦾 Pick de 1 box(es) en cours...
[ORCHESTR] ✅ 1 box(es) récupérée(s)
[ORCHESTR] 
[ORCHESTR] 🚚 Transport vers destination
[ORCHESTR]    → Navigation vers yellow_crate_left
[ORCHESTR] ✅ Arrivé à yellow_crate_left
[ORCHESTR] 🦾 Dépôt des boxes en cours...
[ORCHESTR] ✅ Commande CMD001 préparée !
```

**🎮 Dans Gazebo :**
- Robot va à red_table → attend 3s
- Robot va à blue_table → attend 3s
- Robot va à yellow_crate_left → attend 3s
- Mission terminée

---

### **🛑 SCÉNARIO 4 : ARRÊTER LE ROBOT (MODE SOFT)**

**Quand l'utiliser :** Pause déjeuner, fin de shift

**Étapes :**

1. **Dans Terminal 3**, tape : `4`

2. **Choisis le mode :**
```
🛑 MODES D'ARRÊT:

  [1] 🟡 ARRÊT SOFT
      → Le robot termine sa mission en cours
      → Puis va à la zone start/stop
      → Temps estimé: Variable

  [2] 🔴 ARRÊT IMMÉDIAT
      → Annule TOUT immédiatement
      → Va directement à start/stop

  [0] ❌ Annuler

Votre choix: 1
```

3. **Confirme :**
```
⚠️  CONFIRMATION ARRÊT SOFT

Le robot va:
  ✓ Terminer sa mission en cours
  ✓ Se diriger vers start/stop zone
  ✓ Les missions en attente seront conservées

Confirmer l'arrêt soft ? (o/n): o
```

**✅ Résultat :**
```
✅ Commande d'arrêt soft envoyée
🤖 Le robot va terminer sa mission puis s'arrêter
```

**📊 Dans Terminal 2 :**
```
[QUEUE] 🟡 ARRÊT SOFT DEMANDÉ
[QUEUE]    → Robot va terminer mission en cours
[QUEUE]    → Puis aller à start/stop zone
[QUEUE] 
[QUEUE] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[QUEUE] 📋 FILE DES MISSIONS
[QUEUE] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[QUEUE]   1. 🔴 Retour à start/stop zone (arrêt soft)
[QUEUE]   2. (missions conservées...)
[QUEUE] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

### **🚨 SCÉNARIO 5 : ARRÊT D'URGENCE**

**Quand l'utiliser :** Problème technique, sécurité

**Étapes :**

1. **Dans Terminal 3**, tape : `4` puis `2`

2. **Première confirmation :**
```
🚨 CONFIRMATION ARRÊT IMMÉDIAT

⚠️  ATTENTION: Le robot va:
  ✗ ANNULER la mission en cours
  ✗ VIDER la file des missions
  ✓ Se diriger IMMÉDIATEMENT vers start/stop

🔴 Cette action est IRRÉVERSIBLE !

Confirmer l'arrêt IMMÉDIAT ? (OUI en majuscules): OUI
```

3. **Deuxième confirmation :**
```
⚠️  DERNIÈRE CONFIRMATION
Êtes-vous ABSOLUMENT sûr ? (o/n): o
```

**✅ Résultat :**
```
🚨 Commande d'arrêt IMMÉDIAT envoyée
🤖 Le robot va s'arrêter MAINTENANT
```

**📊 Dans Terminal 2 :**
```
[QUEUE] 🔴 ARRÊT IMMÉDIAT DEMANDÉ
[QUEUE]    → ANNULATION de toutes les missions
[QUEUE]    → Navigation IMMÉDIATE vers start/stop
[QUEUE] 
[QUEUE]    ⚠️  5 mission(s) annulée(s)
[QUEUE] 
[QUEUE] 🚨 ARRÊT D'URGENCE ACTIVÉ
```

---

### **📊 SCÉNARIO 6 : CONSULTER L'HISTORIQUE**

**Étapes :**

1. **Dans Terminal 3**, tape : `5`

2. **Statistiques affichées :**
```
📈 STATISTIQUES:
─────────────────────────────────────────────────────────────
  📦 Total missions: 23
  📥 Arrivages: 5
  🎯 Commandes clients: 15
  🛑 Arrêts: 2
  📝 Modifications stock: 1

FILTRES:
  [1] Tout afficher
  [2] Arrivages uniquement
  [3] Commandes uniquement
  [4] Arrêts uniquement
  [0] Retour
```

3. **Choisis un filtre, par exemple :** `3`

4. **Historique des commandes :**
```
🎯 COMMANDES:
─────────────────────────────────────────────────────────────

  1. 🎯 [20:15:32] CMD001 - Restaurant Le Gourmet: 2x red, 1x blue
  2. 🎯 [20:23:45] CMD002 - Café Central: 3x red, 2x yellow
  3. 🎯 [20:31:12] CMD003 - Hotel Plaza: 1x blue
  ...

OPTIONS:
  [1] Exporter l'historique
  [2] Effacer l'historique
  [0] Retour
```

5. **Pour exporter :** `1`

**✅ Résultat :**
```
✅ Historique exporté: historique_warehouse_20250109_203045.txt
📁 Fichier créé dans le répertoire courant
```

---

## 🎬 **PARTIE 3 : SCÉNARIOS AVANCÉS**

---

### **🔄 SCÉNARIO 7 : ENCHAÎNEMENT AUTOMATIQUE DE MISSIONS**

**Test :**

1. Créer **3 arrivages** coup sur coup
2. Observer dans Terminal 2 que les missions s'ajoutent à la file
3. Observer dans Gazebo que le robot les exécute **une par une automatiquement**

**Exemple de file :**
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📋 FILE DES MISSIONS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  1. 🟢 Tri arrivage (5 boxes)    ← En cours
  2. 🟢 Tri arrivage (3 boxes)    ← En attente
  3. 🟢 Tri arrivage (7 boxes)    ← En attente
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

### **⚡ SCÉNARIO 8 : GESTION DES PRIORITÉS**

**Test :**

1. Créer commande **BASSE priorité**
2. Créer arrivage
3. Créer commande **HAUTE priorité**

**Résultat attendu :**
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📋 FILE DES MISSIONS (TRIÉE PAR PRIORITÉ)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  1. 🔴 CMD002 - Haute priorité
  2. 🟢 CMD001 - Basse priorité
  3. 🟢 Tri arrivage
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

**La commande haute passe devant tout ! ⚡**

---

### **📦 SCÉNARIO 9 : COMMANDE AVEC STOCK INSUFFISANT**

**Test :**

1. Vérifier stock : 3 red disponibles
2. Créer commande : 5 red boxes (priorité HAUTE)

**Résultat :**
```
⚠️  Stock insuffisant ! (disponible: 3)
Voulez-vous quand même commander ? (o/n): o
```

**Si tu confirmes :**
```
[QUEUE] ⚠️  Stock insuffisant pour cette commande
[QUEUE] 💡 Ajout d'une mission de tri prioritaire

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📋 FILE DES MISSIONS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  1. 🔴 Tri urgent pour CMD001      ← Ajouté auto !
  2. 🔴 CMD001 - Client XXX
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

**Le robot trie d'abord l'arrivage, puis prépare la commande ! 🧠**

---

### **🔋 SCÉNARIO 10 : BATTERIE FAIBLE (AUTOMATIQUE)**

**Ce qui se passe :**

1. Laisser tourner 10-15 minutes
2. Batterie descend à ~28%
3. **Interruption automatique !**

**Dans Terminal 2 :**
```
[BATTERY] ⚠️  Batterie faible: 28.5%
[BATTERY] 🔋 Activation mode LOW_BATTERY
[BATTERY] 📢 Publication alerte: LOW_BATTERY

[ORCHESTR] ⚠️  BATTERIE FAIBLE ! Mode LOW_BATTERY activé
[ORCHESTR] 🚨 INTERRUPTION MISSION EN COURS
[ORCHESTR] 🔌 Navigation vers charging_zone

[QUEUE] 🔋 BATTERIE FAIBLE - Robot va charger

[ORCHESTR] ✅ Arrivé à charging_zone
[ORCHESTR] 🔌 Charge en cours...

[BATTERY] 🔋 Batterie: 30.0% (en charge)
[BATTERY] 🔋 Batterie: 40.0% (en charge)
...
[BATTERY] 🔋 Batterie: 95.0% (en charge)
[BATTERY] ✅ Batterie chargée ! (95.0%)

[ORCHESTR] ✅ Charge terminée !
[ORCHESTR] 🔄 Reprise des missions...

[QUEUE] ✅ BATTERIE CHARGÉE - Reprise missions
[QUEUE] 🤖 Robot disponible pour prochaine mission
```

**Le robot reprend automatiquement là où il en était ! 🔋✅**

---

## 🎯 **PARTIE 4 : COMMANDES UTILES**

### **Monitoring système**

```bash
# Voir tous les nodes actifs
ros2 node list

# Voir tous les topics
ros2 topic list

# Écouter la batterie
ros2 topic echo /battery_status

# Voir le stock en temps réel
ros2 topic echo /warehouse/stock_status

# Voir la position du robot
ros2 topic echo /current_zone

# Tester mouvement manuel
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --once

# Forcer niveau batterie (TEST)
ros2 topic pub /battery_override std_msgs/msg/Float32 "data: 25.0" --once
```

---

### **Debugging**

```bash
# Logs détaillés d'un node
ros2 run mission_orchestrator mission_orchestrator --ros-args --log-level debug

# Visualiser TF
ros2 run tf2_tools view_frames
evince frames.pdf

# Check topics Hz
ros2 topic hz /odom
ros2 topic hz /warehouse/stock_status

# Info sur un topic
ros2 topic info /warehouse/execute_mission
```

---

## ❌ **PARTIE 5 : ARRÊTER LE SYSTÈME**

### **Arrêt propre :**

1. **Terminal 3** : Tape `0` (Quitter interface)
2. **Terminal 2** : Ctrl+C
3. **Terminal 1** : Ctrl+C (Gazebo se fermera)

---

## 📋 **PARTIE 6 : CHECKLIST QUOTIDIENNE**

### **Début de journée :**

- [ ] Lancer Terminal 1 (Gazebo)
- [ ] Attendre 60s
- [ ] Lancer Terminal 2 (Warehouse System)
- [ ] Attendre 15s
- [ ] Lancer Terminal 3 (Interface)
- [ ] Modifier stock initial (`1`)
- [ ] Vérifier batterie 100%

### **En cours de journée :**

- [ ] Enregistrer arrivages (`2`)
- [ ] Créer commandes (`3`)
- [ ] Consulter historique (`5`)
- [ ] Rafraîchir affichage (`6`)

### **Fin de journée :**

- [ ] Arrêt soft (`4` → `1`)
- [ ] Exporter historique (`5` → `1`)
- [ ] Quitter système (`0`)

---

## 🎓 **RÉSUMÉ DES TOUCHES CLÉS**

| Touche | Action |
|--------|--------|
| `1` | Modifier stock initial |
| `2` | Nouvel arrivage → Robot trie automatiquement |
| `3` | Commande client → Robot prépare automatiquement |
| `4` | Arrêter robot (soft ou immédiat) |
| `5` | Historique (avec filtres et export) |
| `6` | Rafraîchir affichage |
| `0` | Quitter |

---

## ✅ **TU ES PRÊT À UTILISER TON PROJET !**

**Commence par le SCÉNARIO 2 (Arrivage) pour voir le robot bouger ! 🚀**
