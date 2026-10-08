# Carry My Luggage

Un TurtleBot3 Burger suit son opérateur en portant son bagage : il le verrouille au LiDAR,
le suit à 0,8 m, le cherche s'il le perd et s'arrête sur un geste de la main.
ROS 2 Humble, Python (rclpy + NumPy), simulation Gazebo Classic 11.

Le code suit les deux documents du projet : *Spécifications fonctionnelles* (F1 à F4, S1 à S5,
P, R, E) et *Architecture ROS 2* (4 nœuds, logique pure dans `core/`).

## Arborescence

```
carry_my_luggage_ws/                  # dépôt Git = workspace colcon
├── bags/                             # rosbags des essais (non versionnés)
└── src/carry_my_luggage/             # paquet ament_python
    ├── carry_my_luggage/
    │   ├── leg_detector_node.py      # /scan -> /legs
    │   ├── operator_tracker_node.py  # /legs -> /target (verrouillage + Kalman)
    │   ├── gesture_detector_node.py  # /scan + /target + /odom -> /stop_gesture
    │   ├── follow_manager_node.py    # machine à états, commande, sécurité, bips -> /cmd_vel
    │   ├── sim_operator_node.py      # simulation : fait marcher les jambes dans Gazebo
    │   ├── markers.py                # marqueurs RViz communs
    │   └── core/                     # logique pure, sans ROS, testée avec pytest
    │       ├── clustering.py  kalman.py  controller.py  gesture.py  state_machine.py
    │       ├── geometry.py           # changements de repère, secteurs du scan
    │       └── path_walker.py        # trajet de l'opérateur simulé
    ├── config/params.yaml            # TOUS les seuils et gains (E1)
    ├── config/params_sim.yaml        # surcharges Gazebo
    ├── launch/sim.launch.py  robot.launch.py  rviz.launch.py
    ├── rviz/carry_my_luggage.rviz
    ├── worlds/follow_test.world      # salle avec table et chaise près du parcours (T5)
    ├── models/operator_legs/         # deux cylindres = les jambes de l'opérateur
    └── test/                         # pytest : core/ + flake8 + pep257
```

## Installation

Sur le PC **et** sur le Raspberry Pi (Ubuntu 22.04, ROS 2 Humble déjà installé) :

```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'export LDS_MODEL=LDS-01' >> ~/.bashrc      # ou LDS-02 selon le capteur monté
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc      # le même sur les deux machines
source ~/.bashrc
```

Dépendances sur le PC (simulation comprise) :

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-turtlebot3-msgs ros-humble-turtlebot3 \
                 ros-humble-turtlebot3-gazebo python3-numpy
```

Si `ros-humble-turtlebot3-gazebo` n'est pas disponible, clonez `turtlebot3_simulations`
(branche `humble-devel`) dans `src/`, comme l'indique l'e-Manual ROBOTIS.

Compilation (dans `carry_my_luggage_ws/`) :

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

Sur le Raspberry Pi, inutile d'installer Gazebo :
`rosdep install --from-paths src --ignore-src -y --skip-keys "gazebo_ros gazebo_msgs turtlebot3_gazebo"`.

## Tests

```bash
colcon test --packages-select carry_my_luggage && colcon test-result --verbose
```

57 tests : détection des jambes, Kalman et verrouillage, loi de commande, geste, machine à états,
plus flake8 et pep257. Pour itérer plus vite sur la logique : `cd src/carry_my_luggage && python3 -m pytest test/`.

## Lancer en simulation

```bash
ros2 launch carry_my_luggage sim.launch.py              # gui:=false rviz:=false record:=true
```

Le robot part de (0, 0). L'opérateur simulé l'attend à 0,8 m pendant 10 s (verrouillage + bip),
puis marche à 0,15 m/s sur ~9 m avec deux virages à 90°. Il fait une pause de 10 s au deuxième
virage (F3.5), puis s'arrête au bout du parcours. Les cylindres ne peuvent pas faire le geste
de la main : terminez la mission avec le service d'arrêt.

```bash
ros2 service call /stop_follow std_srvs/srv/Trigger
```

Le parcours, la vitesse et la pause se règlent dans `config/params_sim.yaml`. Pour tester
l'arrêt d'urgence ou un obstacle soudain (T7), posez une boîte devant le robot dans Gazebo.

Sous WSL, les fenêtres Gazebo et RViz s'affichent sous Windows grâce à WSLg. Si la fenêtre
Gazebo reste noire ou plante, relancez avec `export LIBGL_ALWAYS_SOFTWARE=1`. Le launch fixe
`ALSOFT_DRIVERS=null` : sans carte son, OpenAL bloquait le démarrage de Gazebo ~30 s, et le
robot n'apparaissait jamais.

## Lancer sur le robot réel

Sur le Raspberry Pi (en SSH) :

```bash
ros2 launch turtlebot3_bringup robot.launch.py         # pilote LiDAR, roues, odométrie
ros2 launch carry_my_luggage robot.launch.py           # nos 4 nœuds (record:=true pour un rosbag)
```

`ros2 launch carry_my_luggage robot.launch.py bringup:=true` lance les deux d'un coup.
Sur le PC : `ros2 launch carry_my_luggage rviz.launch.py`.

Pendant la mise au point, on peut lancer `robot.launch.py` sur le PC plutôt que sur le Pi.
Mesurez d'abord la fréquence réelle du LiDAR : `ros2 topic hz /scan` (~5 Hz attendus).

## Déroulement d'une mission

| Étape | Ce que fait l'opérateur | Ce que fait le robot | État |
|---|---|---|---|
| 1 | Lance le système | Attend un scan et une batterie correcte | IDLE |
| 2 | Se place face au robot, à 0,5–1 m | Verrouille la personne la plus proche du cône ±30°, 0,4–1,2 m, **1 bip** | ACQUIRE → FOLLOW |
| 3 | Marche lentement | Le suit à 0,8 m, ne recule jamais, s'arrête s'il s'approche à moins de 0,5 m | FOLLOW |
| 4 | S'arrête | S'immobilise en gardant la cible, repart avec lui | FOLLOW |
| — | Disparaît (virage, occlusion) | Après 1 s : s'arrête et pivote vers sa dernière position ; reprend dès qu'il le revoit. Après 15 s : **bip d'erreur**, nouvelle acquisition | SEARCH |
| 5 | S'approche à moins de 0,6 m et met la main devant le LiDAR (< 0,25 m, ±20°) pendant 2 s | S'arrête, **2 bips** : mission terminée | STOPPED |

Sécurité, quel que soit l'état (sauf STOPPED, déjà à l'arrêt) :

- obstacle à moins de 0,15 m devant, ou aucun scan depuis 0,5 s : vitesse nulle (IDLE). Le robot
  repart seul dès que la voie est libre, sans changer de cible ;
- batterie sous 11 V : refuse de démarrer, ou s'arrête (bip batterie) ;
- arrêt d'urgence : vitesse nulle immédiate, **verrouillé** jusqu'à `/start_follow`.

### Services

| Service | Effet |
|---|---|
| `/stop_follow` | Fin de mission, comme le geste (F4.2) |
| `/emergency_stop` | Arrêt d'urgence verrouillé (S3) |
| `/start_follow` | Déverrouille l'arrêt d'urgence ; depuis STOPPED, lance une nouvelle mission |

```bash
ros2 service call /emergency_stop std_srvs/srv/Trigger
```

### Topics

| Topic | Type | Rôle |
|---|---|---|
| `/legs` | `geometry_msgs/PoseArray` | Centres des jambes candidates (repère du LiDAR) |
| `/target` | `geometry_msgs/PointStamped` | Position prédite de l'opérateur (repère `odom`), publiée à 20 Hz tant qu'il est suivi |
| `/stop_gesture` | `std_msgs/Empty` | Geste d'arrêt reconnu |
| `/follow_state` | `std_msgs/String` | IDLE, ACQUIRE, FOLLOW, SEARCH ou STOPPED |
| `/cmd_vel` | `geometry_msgs/Twist` | Publié **uniquement** par `follow_manager`, à 20 Hz |
| `/target_polar` | `geometry_msgs/Vector3Stamped` | x = distance, y = angle, z = erreur de distance (pour PlotJuggler) |
| `/follow_markers` | `visualization_msgs/MarkerArray` | Jambes, personnes, cible, zone du geste, zone de sécurité, état |

## Enregistrer un essai (E4)

`record:=true` sur `sim.launch.py` ou `robot.launch.py` enregistre dans `bags/` (chemin relatif
au dossier d'où l'on lance). Pour tracer la distance de suivi (T2, T3), ouvrez le bag dans
PlotJuggler et affichez `/target_polar/vector/x` et `/cmd_vel/linear/x`.

Pour régler le suivi sur un bag réel sans `follow_manager`, passez `auto_acquire: true` à
`operator_tracker` : il verrouille alors seul la première personne devant le robot.

## Réglages

Tout est dans `config/params.yaml`, commenté avec l'exigence correspondante. Les réels s'écrivent
avec un point (`15.0`, pas `15`), sinon le nœud refuse de démarrer. À régler en priorité :

- `k_v`, `k_w` : réactivité du suivi. En marche stabilisée, le robot traîne de `v / k_v`.
- `leg_width_min/max`, `cluster_jump_distance` : à ajuster sur des rosbags réels (jambes de
  pantalon large, jupe…).
- `gesture_distance`, `stop_gesture_duration` : confort du geste (question ouverte des specs).
- `battery_critical_voltage`, `sound_*` : à vérifier sur votre OpenCR.

## Choix faits et écarts avec les documents

Tous ces points sont réglables ou facilement réversibles.

1. **`k_v = 1,5`** au lieu de 0,5. Avec un régulateur proportionnel, le robot traîne de
   `v / k_v` : 0,30 m à 0,15 m/s avec 0,5, ce qui casse F3.1 (±0,2 m). Avec 1,5, l'erreur
   moyenne mesurée en simulation 2D est de 0,13 m.
2. **Service `/start_follow`** ajouté : il déverrouille l'arrêt d'urgence et relance une
   mission après STOPPED, sans relancer les nœuds.
3. **Topic `/target_polar`** ajouté, pour les courbes du rapport.
4. **`gesture_detector` lit aussi `/odom`**, pour la condition « robot à l'arrêt » du geste.
5. **Danger → IDLE, puis reprise automatique.** Obstacle et LiDAR muet ramènent en IDLE ; dès
   que c'est fini, IDLE → ACQUIRE → FOLLOW dans le même cycle si la cible est toujours suivie
   (scénario nominal, étape 4). L'arrêt d'urgence, lui, reste verrouillé, et STOPPED reste final.
6. **Le geste et `/stop_follow` sont acceptés dans tous les états**, pas seulement en FOLLOW :
   une main trop près (< 0,15 m) déclenche d'abord l'arrêt de sécurité, et le geste doit quand
   même terminer la mission.
7. **Clustering** : un nouveau cluster s'ouvre aussi quand plus de 2 rayons sans écho séparent
   deux points. Sans cela, deux jambes écartées de moins de 10 cm devant un fond hors de portée
   fusionnaient en un seul cluster trop large, et le verrouillage échouait.
8. **Jambe seule** : acceptée seulement à moins de 0,3 m de la prédiction (`single_leg_gate`),
   contre 0,5 m pour une paire, pour qu'un pied de table voisin ne vole pas la cible (R2).
9. **Verrouillage confirmé sur 3 scans consécutifs** (≈ 0,6 s), pour ne pas verrouiller un
   faux positif d'un seul scan.
10. Deux modules ajoutés dans `core/` : `geometry.py` et `path_walker.py`.

## Points d'attention sur le robot réel

- **LDS-02** : sa portée minimale (0,16 m) dépasse `safety_distance` (0,15 m), donc la règle S2
  ne se déclencherait jamais. Passez `safety_distance` à 0,18 ; `follow_manager` affiche un
  avertissement au démarrage. La main du geste doit alors se trouver entre 0,16 et 0,25 m.
- **Les distances de sécurité se mesurent depuis le centre du LiDAR.** L'avant du Burger est à
  environ 7 cm de ce centre.
- **Le bagage ne doit pas masquer le LiDAR**, qui est sur le plateau supérieur du Burger : il faut
  un plateau surélevé. Si ses entretoises sont vues à plus de `range_min`, elles seront prises
  pour un obstacle permanent.
- **Bips** : les valeurs de `turtlebot3_msgs/srv/Sound` (1 ON, 2 LOW_BATTERY, 3 ERROR,
  4 BUTTON1, 5 BUTTON2) jouent des mélodies définies par le firmware OpenCR. Vérifiez qu'elles
  conviennent et changez `sound_*` sinon.
- **Hors périmètre** : la ré-identification. Si une autre personne passe très près de
  l'opérateur, le suivi peut basculer sur elle.

## Correspondance exigences → code

| Exigence | Où |
|---|---|
| F1.1 pose de départ | `follow_manager_node.on_odom` (marqueur `home`) |
| F1.2, F1.3 verrouillage, bip | `core/kalman.OperatorTracker._acquire`, `follow_manager_node.on_transition` |
| F2.1 jambes | `core/clustering.detect_legs` |
| F2.2 à F2.4 Kalman, perte, association | `core/kalman.py` |
| F2.3, F2.5 recherche, délai | `core/controller.search_command`, `core/state_machine.py` |
| F3.1 à F3.5 suivi | `core/controller.follow_command` |
| F4.1 geste | `core/gesture.py`, `gesture_detector_node.py` |
| F4.2, S3 services | `follow_manager_node.on_stop_follow`, `on_emergency_stop` |
| S1 à S5 sécurité | `follow_manager_node.safety_status`, `publish_cmd` |
| P1, P2 | boucle de commande à 20 Hz, `/target` republié dès chaque scan |
| E1 à E4 | `config/`, `launch/`, `/follow_markers` + `rviz/`, `record:=true` |

## Ce qui a été vérifié

- `colcon build` et les 57 tests passent sous ROS 2 Humble (Ubuntu 22.04, WSL).
- Les 4 nœuds (`robot.launch.py`) ont tourné face à un simulateur 2D cinématique (LiDAR LDS-01
  360 rayons à 5 Hz, bruit 1 cm, table et chaise près du parcours) sur un scénario complet :
  - verrouillage en 0,4 s ;
  - suivi sur 9 m avec deux virages et une pause de 10 s, sans nouvelle acquisition ;
  - erreur de distance moyenne de 0,13 m (max 0,16 m) ;
  - occlusion de 2 s : opérateur retrouvé dès qu'il réapparaît ;
  - geste reconnu en 2,3 s, puis 2 bips ;
  - arrêt d'urgence, coupure du LiDAR, obstacle à 0,13 m, `/stop_follow`.
- `sim.launch.py` dans Gazebo 11 (sans interface), mesures faites avec la vérité terrain de Gazebo :
  - verrouillage 0,6 s après ACQUIRE ;
  - parcours complet de 9,2 m, deux virages et pause de 10 s, sans aucune perte ni saut de
    cible sur la table ou la chaise ;
  - erreur de distance moyenne de +0,15 m (max 0,21 m) ;
  - `/stop_follow` → STOPPED.
- **Pas encore testé** : le geste et l'occlusion dans Gazebo (seulement en 2D), puis le robot
  réel. Ce sont les prochaines étapes de la démarche (T1 à T8).
