# Uart Commmunication Node

## Description

Ce package ROS2 contient un noeud permettant de recevoir et envoyer des
messages aux cartes connectées via liaison série USB.

Son but est d'identifer le message qui vient d'être reçu via liaison séries
et de transferer son contenu dans le bon topic. Ce noeud permet donc de faire
le pont entre un message envoyé par une des cartes PIC et l'environnement ROS.
De plus des messages peuvent être envoyés sur le topic 'action', ce qui permet
d'envoyer des chaines de caractères aux cartes sélectionnées.

Chaque trames envoyées par le PIC est constitué d'une première chaine de
caractère permettant de spécifier la nature du message qui va suivre.
A l'aide des fichiers de configuration présent dans `/ressource`
le noeud est donc capable de connaitre la structure du message 
qui va suivre et donc de correctement rediriger les données qu'il contient.

## Interfaces
### Subscribers
Ce noeud dispose des Subscribers suivants :
| Nom | Type | Description |
| --- | ---- | ----------- |
| Inject raw data | String | Permet d'injecter directement des trames brutes dans le système. **Activé uniquement si `simulation_mode=True`** |
### Publishers
Ce noeud est assez particulier sur ce point puisque son but est de créer
dynamiquement des Publishers en fonction du fichier de configuration
utilisé. En effet c'est lui qui défini les trames qui peuvent être
reçues, et par conséquent les publishers nécéssaires à la publication
des messages ROS.

### Paramètres
Les paramètres disponibles pour le noeud `uart_receiver_node` sont les suivants:

| Nom | Type | Valeur Par Défaut | Description |
| --- | ---- | ----------------- | ----------- |
| `config_file` | str | `default_cdf_2023` | La chaine de caractère spécifiée ici permet de définir le fichier de configuration à utiliser pour interpreter les trames recues.
| `debug_mode`     | bool | False | Permet de rediriger les trames brutes reçues directement dans un topic |
| `raw_received_pic_msg_topic` | str | `/robot_x/raw_received_pic_msg` | Définit le nom du topic visant à recevoir les trames brutes reçues lorque le mode debug est activé |
| `pic_msg_freq` | int | `9` | Fréquence à laquelle le script met à jours les données disponibles dans les buffers d'entrée des ports série |
| `simulation_mode` | bool | False | Permet d'indiquer si l'on souhaite utiliser le topic `pic_message_inject_topic` afin d'injecter directement des trames brutes dans le système |
| `pic_message_inject_topic` | str | `/robot_x/pic_message_inject` | Définit le nom du topic permettant d'injecter directement des trames brutes dans le système. Sert principalement pour tester le bon fonctionnement du fichier de configuration utilisé |

## Configuration et Ajout de cartes
Au début du fichier `uart_sender_node.py` un dictionnaire nommé `robot_cards_cfg`
contient la description des cartes qui sont connectées a la raspberry.
La structure de ce dictionnaire est le suivant:
```python
{
	"nom_de_la_carte1": ["path_vers_device (si connu)", baudrate, "methode_indentification"],
	...
}
```

## Méthodes d'indentification des cartes
Deux méthodes sont disponible afin d'associer chaque port série à un type de carte (`motor` ou `user`)

1. Identification `version_request`:
   Dans ce cas de figure on liste tous les ports dont le nom contient
   les mots-clefs `ttyUSB` ou `ttyAMA` et envoi la commande VERSION.
   La carte concernée va alors donner son type et ainsi de suite.
2. Identification `hardware`:
   L'Identification `hardware` nécéssite une configuration des règles `udev`.
   Elle laisse le système automatiquement associer le port série à un symlink
   ayant un nom particulier. Le device `/dev/ttyAMA0` correspondra ici à la
   carte `user`, et le device `/dev/ttyUSBMotorCard` correspondra à la carte
   `moteur`.
   Plus de détails sur la définition des règles `udev` [ici](http://www.lukylx.org/udev-rules.html).

## Configuration:
### Fichiers de config et dictionnaire `msg_dict`
Toutes les configuration disponibles se trouvent dans le répertoire `resources`.

Chaque fichier de configuration contient le dictionnaire `msg_dict`
qui définit le squelette des trames pouvant être reçues par USB.

Ici, chaque clé de ce dictionnaire désigne la chaine de caractère
qui permet d'identifier le type de trame qui est en train d'être traitée.
Par exemple si la trame suivante est recue par notre noeud :
```abcdef,0,1```
Ce dernier saura que son squelette est défini dans la clé `abcdef` du
dictionnaire `msg_dict`.

### Définition d'une trame
Dans notre cas une trame est une chaine de caractère séparé par le symbole
spécial retour à la ligne `\n`.
Cette chaine de caractère peut elle aussi être décomposée en éléments tous
séparés par des virgules, et dans le premier élément décrit le type de
trame émis, comme nous avons pu le dire précédemment.

**Remarque:** Une trame ne correspond pas toujours à un unique type de
message ROS,ainsi, pour une unique trame reçu, plusieurs messages ROS
pourront être envoyés.

Pour chaques trame on associe un dictionnaire qui contiendra les messages
ROS à envoyer à partir des informations contenues dans la trame.
Les valeurs données aux clefs importent peu mais permettent simplement
de s'y retrouver.

Cependant les valeurs associées à ces clefs contient toutes les informations
concerant le message et sont de la forme suivante:  
    - `topic_rosparam` : Définie le nom du paramètre pouvant servir à spécifier le topic sur lequel le message sera publié.  
    - `topic_default` : Définie le nom du topic par défaut sur lequel le message sera publié.  
    - `msg_type` : Définie le type de message qui sera publié.  
    - `publish_on_change` : Définit si le message sera publié quand il change ou non.  
    - `structure` : Définit la structure du message. Cette clé est elle aussi associée a une valeur qui est un dictionnaire (oui je sais ca fait beaucoup). Chaque clé de ce dictinnaire décrit la position dans la trame de la valeur que l'on souhaite traiter. A cette clé on associe deux paramètre que sont `value_expression` et `path`.
    Ils décrivent respectivement l'expression qui sera appliquée à la valeur de la trame ainsi que le chemin vers la variable qui contiendra cette valeur.

## Exemple
Si l'on souhaite traiter la trame suivante :
```js
robot_pose,<angle_x>,<pos_x>,<angle_y>,<pos_y>,<angle_z>,<pos_z>\n
```
Et publier ces informations dans deux topic différents, un
décrivant la position du robot et l'autre son orientation.
Nous ferons ici l'hypothèse que ces deux topics utilisent
le même type de message, `Vector3`, permettant de stoker 3 flottant.
La formulation de cette configuration est par conséquent la suivante :
```python
msg_dict = {
    'robot_pose': {
        'position msg': {
            'topic_rosparam': '~pos_topic',
            'topic_default': 'pos',
            'msg_type': 'Vector3',
            'publish_on_change': True,
            'structure': {
                0: {
                    "value_expression": lambda x: float(x),
                    "path": "x"
                },
                2: {
                    "value_expression": lambda x: float(x),
                    "path": "y"
                },
                4: {
                    "value_expression": lambda x: float(x),
                    "path": "z"
                }
            }
        },
        'angle msg': {
            'topic_rosparam': '~ang_topic',
            'topic_default': 'ang',
            'msg_type': 'Vector3',
            'publish_on_change': True,
            'structure': {
                1: {
                    "value_expression": lambda x: float(x),
                    "path": "x"
                },
                3: {
                    "value_expression": lambda x: float(x),
                    "path": "y"
                },
                5: {
                    "value_expression": lambda x: float(x),
                    "path": "z"
                }
            }
        }
    }
}
```
