# Carry My Luggage

Un TurtleBot3 Burger suit son opérateur en portant son bagage : il le verrouille au LiDAR,
le suit à 0,8 m, le cherche s'il le perd et s'arrête sur un geste de la main.
ROS 2 Humble, Python (rclpy + NumPy), simulation Gazebo Classic 11.

Le code suit les deux documents du projet : *Spécifications fonctionnelles* (F1 à F4, S1 à S5,
P, R, E) et *Architecture ROS 2* (4 nœuds, logique pure dans `core/`). Deux fonctionnalités
ont été ajoutées au sujet : le **contournement des obstacles** pendant le suivi (VFH) et le
**retour autonome au point de départ** une fois le bagage livré (voir « Fonctionnalités
ajoutées »).

## Arborescence

```
carry_my_luggage_ws/                  # dépôt Git = workspace colcon
├── bags/                             # rosbags des essais (non versionnés)
└── src/carry_my_luggage/             # paquet ament_python
    ├── carry_my_luggage/
    │   ├── leg_detector_node.py      # /scan -> /legs
    │   ├── operator_tracker_node.py  # /legs -> /target (verrouillage + Kalman)
    │   ├── gesture_detector_node.py  # /scan + /target + /odom -> /stop_gesture
    │   ├── follow_manager_node.py    # machine à états, commande, évitement, retour, sécurité -> /cmd_vel
    │   ├── sim_operator_node.py      # simulation : déplace les jambes dans Gazebo
    │   ├── operator_teleop_node.py   # simulation : pilote l'opérateur avec les flèches
    │   ├── markers.py                # marqueurs RViz communs
    │   └── core/                     # logique pure, sans ROS, testée avec pytest
    │       ├── clustering.py  kalman.py  controller.py  gesture.py  state_machine.py
    │       ├── avoidance.py          # évitement d'obstacles (histogramme polaire VFH)
    │       ├── geometry.py           # changements de repère, secteurs du scan
    │       ├── path_walker.py        # parcours scripté de l'opérateur simulé
    │       └── teleop.py             # clavier -> déplacement de l'opérateur simulé
    ├── config/params.yaml            # TOUS les seuils et gains (E1)
    ├── config/params_sim.yaml        # surcharges Gazebo
    ├── launch/sim.launch.py  robot.launch.py  rviz.launch.py
    ├── rviz/carry_my_luggage.rviz
    ├── worlds/follow_test.world      # salle avec table et chaise près du parcours (T5)
    ├── models/operator_legs/         # deux cylindres = les jambes de l'opérateur
    ├── models/obstacle_box/          # carton de 40 cm pour tester l'évitement
    └── test/                         # pytest : core/ + flake8 + pep257
```

## Récupérer le projet sur un autre ordinateur (ex. PC du lycée)

**1. Vérifier ROS 2 et les paquets TurtleBot3** (Ubuntu 22.04) :

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep -E "^(turtlebot3_bringup|turtlebot3_msgs|turtlebot3_gazebo|gazebo_ros)$"
```

Les 4 noms doivent s'afficher (`turtlebot3_gazebo` et `gazebo_ros` ne servent qu'à la
simulation). S'il en manque, installez-les (section Installation) ou, sans droits
administrateur, demandez-le au responsable de la salle.

**2. Cloner et compiler** (le dépôt est public : pas besoin de token pour cloner) :

```bash
git clone https://github.com/MomoFall/ROS_CARRY.git ~/carry_my_luggage_ws
cd ~/carry_my_luggage_ws
colcon build --symlink-install
source install/setup.bash
```

Plus tard, pour récupérer les dernières modifications : `git pull` puis
`colcon build --symlink-install`. Pour pousser depuis cet ordinateur, il faut votre token GitHub
et une identité Git dans le dépôt (`git config user.name ...` / `git config user.email ...`).

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

80 tests : détection des jambes, Kalman et verrouillage, loi de commande, geste, machine à états
(retour compris), évitement d'obstacles, téléopération de l'opérateur simulé, plus flake8 et pep257. Pour itérer plus vite sur la logique : `cd src/carry_my_luggage && python3 -m pytest test/`.

## Lancer en simulation

```bash
ros2 launch carry_my_luggage sim.launch.py              # gui:=false rviz:=false record:=true
```

Le robot part de (0, 0). L'opérateur simulé l'attend immobile à 0,8 m devant lui ; le robot le
verrouille (bip) en moins d'une seconde.

**Piloter l'opérateur au clavier** (par défaut, comme `turtle_teleop_key` de turtlesim), dans un
second terminal :

```bash
ros2 run carry_my_luggage operator_teleop
```

| Touche | Effet |
|---|---|
| ↑ / ↓ | avancer / reculer |
| ← / → | tourner sur place à gauche / à droite |
| espace | s'arrêter tout de suite |
| + / - | vitesse de marche (0,15 m/s au départ, entre 0,05 et 0,5 m/s) |
| Ctrl+C | quitter |

Comme dans turtlesim, l'opérateur marche tant que la touche est maintenue et s'arrête 0,7 s
après l'avoir relâchée (`teleop_timeout` dans `params_sim.yaml`). Pour suivre un trajet en
courbe, alternez ↑ et ← / →. Au-delà de 0,22 m/s, le robot ne peut plus suivre et finit par
perdre l'opérateur : c'est le moyen de tester la recherche (SEARCH).

**Parcours scripté** (tests reproductibles T2, T3) : `sim.launch.py operator:=path`. L'opérateur
attend 10 s, puis marche à 0,15 m/s sur ~9 m avec deux virages à 90° et une pause de 10 s
(F3.5). Le parcours se règle dans `config/params_sim.yaml`.

Les cylindres ne peuvent pas faire le geste de la main : terminez la mission avec le service
d'arrêt.

```bash
ros2 service call /stop_follow std_srvs/srv/Trigger
```

Pour tester l'arrêt d'urgence ou un obstacle soudain (T7), posez une boîte devant le robot dans
Gazebo.

**Tester l'évitement et le retour** : `sim.launch.py obstacle:=true` ajoute un carton de 40 cm
en (2, 0) (`obstacle_x:=… obstacle_y:=…` pour le déplacer). Faites passer l'opérateur juste à
côté du carton : le robot le contourne au lieu de s'arrêter. Puis `/stop_follow` : 5 s plus
tard, le robot rentre en (0, 0) en contournant de nouveau le carton. Dans RViz, le trait au-dessus
du robot montre le cap choisi (vert : droit vers le but, orange : détour) et les points rouges les
directions bloquées.

Sous WSL, les fenêtres Gazebo et RViz s'affichent sous Windows grâce à WSLg. Si la fenêtre
Gazebo reste noire ou plante, relancez avec `export LIBGL_ALWAYS_SOFTWARE=1`. Le launch fixe
`ALSOFT_DRIVERS=null` : sans carte son, OpenAL bloquait le démarrage de Gazebo ~30 s, et le
robot n'apparaissait jamais.

## Lancer sur le robot réel (pas à pas)

Pour les premiers essais, nos 4 nœuds tournent sur le PC et le robot ne lance que son
`turtlebot3_bringup` : on corrige et on relance sans toucher au robot. Une fois au point, on
les déplace sur le Raspberry Pi (étape 6).

**1. Réseau et variables, sur le PC et sur le robot**

- PC et robot sur le même réseau Wi-Fi.
- Le **même `ROS_DOMAIN_ID`** des deux côtés, et **différent de celui des autres groupes** de la
  salle : sinon vos nœuds commandent le robot du voisin (et inversement). Lisez celui du robot
  (`grep ROS_DOMAIN_ID ~/.bashrc` en SSH), puis, sur le PC, dans chaque terminal :
  `export ROS_DOMAIN_ID=<la même valeur>`.
- `export TURTLEBOT3_MODEL=burger` sur le PC ; sur le robot, `TURTLEBOT3_MODEL` et `LDS_MODEL`
  sont normalement déjà dans son `~/.bashrc`.

**2. Démarrer le robot** (terminal SSH) :

```bash
ssh ubuntu@<IP_DU_ROBOT>
ros2 launch turtlebot3_bringup robot.launch.py
```

**3. Vérifier depuis le PC** :

```bash
ros2 topic hz /scan                                  # ~5 Hz attendus
ros2 topic echo /scan --once --field range_min       # 0.12 : LDS-01 ; 0.16 : LDS-02
```

Avec un LDS-02, mettez `safety_distance: 0.18` dans `src/carry_my_luggage/config/params.yaml`,
sinon l'arrêt devant un obstacle (S2) ne peut pas se déclencher. Grâce à `--symlink-install`,
pas besoin de recompiler : relancer suffit.

**4. Premier essai, roues dans le vide** : posez le robot sur une boîte, puis sur le PC

```bash
ros2 launch carry_my_luggage robot.launch.py
ros2 launch carry_my_luggage rviz.launch.py          # dans un second terminal
```

Placez-vous face au robot, à 0,5–1 m : il bipe et passe en FOLLOW (état affiché au-dessus du
robot dans RViz). Reculez : les roues tournent ; approchez à moins de 0,5 m : elles s'arrêtent.
Vérifiez aussi qu'un Ctrl+C sur `robot.launch.py` arrête bien les roues.

**5. Au sol**, dans un espace dégagé, avec un équipier qui garde ce terminal prêt :

```bash
ros2 service call /emergency_stop std_srvs/srv/Trigger
```

Relance après un arrêt d'urgence : `ros2 service call /start_follow std_srvs/srv/Trigger`.
Enchaînez ensuite les tests T1 à T8 des spécifications, en ajoutant `record:=true` à
`robot.launch.py` pour enregistrer chaque essai dans `bags/`.

Si RViz affiche des erreurs de transformation « into the future », les horloges du PC et du
robot sont décalées : synchronisez-les (par exemple avec `chrony` sur les deux machines).

**6. Plus tard, nœuds sur le Raspberry Pi** (la boucle de commande ne dépend alors plus du
Wi-Fi) : clonez et compilez le dépôt sur le robot (voir Installation, sans Gazebo), puis
`ros2 launch carry_my_luggage robot.launch.py bringup:=true` en SSH. Le PC ne lance plus que RViz.

## Déroulement d'une mission

| Étape | Ce que fait l'opérateur | Ce que fait le robot | État |
|---|---|---|---|
| 1 | Lance le système | Attend un scan et une batterie correcte | IDLE |
| 2 | Se place face au robot, à 0,5–1 m | Verrouille la personne la plus proche du cône ±30°, 0,4–1,2 m, **1 bip** | ACQUIRE → FOLLOW |
| 3 | Marche lentement | Le suit à 0,8 m en contournant les obstacles, ne recule jamais, s'arrête s'il s'approche à moins de 0,5 m | FOLLOW |
| 4 | S'arrête | S'immobilise en gardant la cible, repart avec lui | FOLLOW |
| — | Disparaît (virage, occlusion) | Après 1 s : s'arrête et pivote vers sa dernière position ; reprend dès qu'il le revoit. Après 15 s : **bip d'erreur**, nouvelle acquisition | SEARCH |
| 5 | S'approche à moins de 0,6 m et met la main devant le LiDAR (< 0,25 m, ±20°) pendant 2 s | S'arrête, **2 bips** : bagage livré | STOPPED |
| 6 | Reprend son bagage | 5 s plus tard, **1 bip**, rentre seul au point de départ en contournant les obstacles | RETURN |
| 7 | — | Arrivé, se remet dans son orientation de départ, **2 bips** | HOME |

Sécurité, quel que soit l'état :

- obstacle à moins de 0,15 m devant (malgré l'évitement), ou aucun scan depuis 0,5 s : vitesse
  nulle. Pendant le suivi, le robot passe en IDLE et repart seul dès que la voie est libre, sans
  changer de cible ; pendant le retour, il attend simplement sur place ;
- batterie sous 11 V : refuse de démarrer, ou s'arrête (bip batterie) ;
- arrêt d'urgence : vitesse nulle immédiate, **verrouillé** jusqu'à `/start_follow`.

### Services

| Service | Effet |
|---|---|
| `/stop_follow` | Fin de mission, comme le geste (F4.2) ; pendant le retour, l'annule (le robot reste sur place) |
| `/emergency_stop` | Arrêt d'urgence verrouillé (S3) |
| `/start_follow` | Déverrouille l'arrêt d'urgence ; depuis STOPPED ou HOME, lance une nouvelle mission qui part de la position actuelle |

```bash
ros2 service call /emergency_stop std_srvs/srv/Trigger
```

### Topics

| Topic | Type | Rôle |
|---|---|---|
| `/legs` | `geometry_msgs/PoseArray` | Centres des jambes candidates (repère du LiDAR) |
| `/target` | `geometry_msgs/PointStamped` | Position prédite de l'opérateur (repère `odom`), publiée à 20 Hz tant qu'il est suivi |
| `/stop_gesture` | `std_msgs/Empty` | Geste d'arrêt reconnu |
| `/follow_state` | `std_msgs/String` | IDLE, ACQUIRE, FOLLOW, SEARCH, STOPPED, RETURN ou HOME |
| `/cmd_vel` | `geometry_msgs/Twist` | Publié **uniquement** par `follow_manager`, à 20 Hz |
| `/target_polar` | `geometry_msgs/Vector3Stamped` | x = distance, y = angle, z = erreur de distance (pour PlotJuggler) |
| `/follow_markers` | `visualization_msgs/MarkerArray` | Jambes, personnes, cible, zone du geste, zone de sécurité, état, cap d'évitement, point de départ |

## Enregistrer un essai (E4)

`record:=true` sur `sim.launch.py` ou `robot.launch.py` enregistre dans `bags/` (chemin relatif
au dossier d'où l'on lance). Pour tracer la distance de suivi (T2, T3), ouvrez le bag dans
PlotJuggler et affichez `/target_polar/vector/x` et `/cmd_vel/linear/x`.

Pour régler le suivi sur un bag réel sans `follow_manager`, passez `auto_acquire: true` à
`operator_tracker` : il verrouille alors seul la première personne devant le robot.

## Fonctionnalités ajoutées

Rédigées comme les exigences des spécifications, pour pouvoir les y reporter.

### F5 — Contourner un obstacle pendant le suivi (VFH)

- **F5.1** Les points du scan situés entre le robot et l'opérateur (à moins de 1 m, et jamais
  au-delà de l'opérateur) sont des obstacles. Les points à moins de 0,35 m de l'opérateur sont
  ses jambes et sont ignorés.
- **F5.2** Chaque obstacle, grossi du rayon du robot (0,11 m) et d'une marge (0,08 m), bloque
  les secteurs angulaires de 5° dans lesquels le robot le heurterait (histogramme polaire).
- **F5.3** Si la direction de l'opérateur est libre, le robot y va tout droit. Sinon, il prend
  le secteur libre le plus proche de l'opérateur (détour de 100° au plus), en gardant de
  préférence le côté choisi au cycle précédent pour ne pas hésiter.
- **F5.4** La distance à l'opérateur règle toujours la vitesse (F3.2), et l'évitement ne sert
  que si le robot doit avancer : arrêté à 0,8 m, il reste tourné vers l'opérateur.
- **F5.5** Si toutes les directions sont bloquées, le robot attend, tourné vers l'opérateur.
  L'arrêt à 0,15 m (S2) reste le dernier recours.
- **Acceptation** (T9) : un carton de 40 cm entre le robot et l'opérateur est contourné sans
  arrêt de sécurité ni perte de l'opérateur, le robot restant à plus de 10 cm du carton.

### F6 — Rentrer seul au point de départ

- **F6.1** Une fois le bagage livré (STOPPED par le geste ou `/stop_follow`), le robot attend
  5 s puis part vers la pose enregistrée au lancement (F1.1) : état RETURN, 1 bip.
- **F6.2** Il y va à 0,15 m/s au plus en contournant les obstacles (F5), à l'odométrie.
- **F6.3** À moins de 10 cm du point de départ, il se remet dans son orientation de départ
  (± 5°), s'arrête et bipe deux fois : état HOME.
- **F6.4** Un danger met le retour en pause sans l'annuler ; `/stop_follow` l'annule ; au-delà
  de 120 s, il est abandonné (bip d'erreur).
- **F6.5** `/start_follow` depuis HOME lance une nouvelle mission, qui part de là.
- **Acceptation** (T10) : après un suivi de 3 à 10 m, le robot revient à moins de 15 cm de
  son point de départ (en simulation : moins de 10 cm).

Les deux fonctionnalités se désactivent dans `params.yaml` (`avoidance: false`,
`return_home: false`) pour retrouver exactement le comportement des spécifications d'origine.

**Limites à connaître** :

- **Hauteur :** le LiDAR ne voit qu'à sa hauteur, à environ 18 cm du sol. Un obstacle plus bas
  (marche, sac posé par terre) n'est ni contourné ni détecté par S2.
- **Approche locale :** VFH ne connaît pas le plan de la salle. Il contourne un obstacle isolé,
  mais un cul-de-sac (obstacle en U) peut le bloquer.
- **Passages étroits :** il faut au moins 2 × 0,19 = 0,38 m de large pour passer.
- **Dérive de l'odométrie :** le retour se fait à l'odométrie, qui dérive sur le vrai robot
  (glissement des roues, surtout dans les virages). Comptez quelques centimètres d'erreur par
  mètre parcouru ; sans carte, rien ne la corrige.
- **Bagage qui dépasse :** s'il dépasse du robot, augmentez `robot_radius`.

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
   (scénario nominal, étape 4). L'arrêt d'urgence, lui, reste verrouillé. Avec les
   fonctionnalités ajoutées, S2 n'est plus le comportement normal devant un obstacle (il est
   contourné, F5) et STOPPED n'est plus final (retour au départ, F6).
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

## Dépannage

**Gazebo s'ouvre, mais ni robot ni opérateur** (`spawn_entity` attend sans fin, et
`SetEntityState: entity [operator_legs] does not exist` défile en boucle). Deux causes :

1. **Un Gazebo d'un lancement précédent tourne encore**, par exemple après une fenêtre fermée
   ou un Ctrl+C. Il garde le port de Gazebo, le nouveau ne démarre pas. `sim.launch.py` le
   détecte et s'arrête avec un message ; pour le fermer :

   ```bash
   killall -9 gzserver gzclient
   ```

2. **Gazebo attend sa base de modèles en ligne**, que le réseau de l'établissement bloque.
   `sim.launch.py` la désactive (`GAZEBO_MODEL_DATABASE_URI` vide) et donne lui-même les dossiers
   de modèles locaux, TurtleBot3 compris. Si le problème apparaît avec une ancienne version du
   dépôt, mettez-la à jour : `git pull` puis `colcon build --symlink-install`.

**Fenêtre Gazebo noire ou qui plante** (WSL, carte graphique) : `export LIBGL_ALWAYS_SOFTWARE=1`
avant de lancer.

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
| F5 évitement (VFH) | `core/avoidance.py`, `follow_manager_node.avoid_heading` |
| F6 retour au départ | `core/state_machine.py` (RETURN, HOME), `core/controller.goto_command`, `follow_manager_node.return_command` |
| S1 à S5 sécurité | `follow_manager_node.safety_status`, `publish_cmd` |
| P1, P2 | boucle de commande à 20 Hz, `/target` republié dès chaque scan |
| E1 à E4 | `config/`, `launch/`, `/follow_markers` + `rviz/`, `record:=true` |

## Ce qui a été vérifié

- `colcon build` et les 80 tests passent sous ROS 2 Humble (Ubuntu 22.04, WSL).
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
- Pilotage au clavier : `operator_teleop` testé dans un pseudo-terminal (flèches, + / -,
  Ctrl+C). Dans Gazebo, avec des appuis simulés, le robot suit l'opérateur entre 0,80 et 0,97 m,
  virages compris, sans le perdre.
- Évitement et retour, dans Gazebo avec le carton en (2, 0), sur un trajet où la ligne droite
  robot → opérateur traverse le carton :
  - pendant le suivi, le robot fait un détour sur 18 des 103 cycles enregistrés. Il reste à
    0,20 m au moins de la surface du carton (environ 11 cm de jeu sur le côté), sans arrêt de
    sécurité ni perte de l'opérateur ;
  - au retour, le détour dure 44 cycles sur 108. Le robot arrive à 8,5 cm du point de départ,
    orienté à -2,2° de son orientation de départ, en 26 s.
- **Pas encore testé** : le geste et l'occlusion dans Gazebo (seulement en 2D), puis le robot
  réel. Ce sont les prochaines étapes de la démarche (T1 à T10).
