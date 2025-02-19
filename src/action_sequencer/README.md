# Action Sequencer ROS package

## Attention !!
Ce package nécéssite de nouveaux messages définis sur la branche dev
de du package [cdf_msgs](https://github.com/7Robot/cdf_msgs).

## Description
Ce package à pour but principal d'executer différentes tâches appelées
"séquence d'actions". Ces séquences d'actions représentent des tâches d'apparence
simple mais qui sont en réalité complexe à executer, souvent par le nombre
assez important d'actions plus élémentaires qui les composent.
Par exemple prenons la tâche suivante :
> Range le cube rouge dans la boite

Cette tâche peut paraitre simple à première vue, mais est en réalité composée
de nombreuses actions plus petites telles que :

> - Identifie la position du cube rouge
> - Positionne toi au dessus de cette position
> - Attrape le cube
> - ...

Ce noeud à donc pour but d'être couplé avec un noeud "cerveau", donnant des
ordres (les tâches) à effectuer.
L'interet derrière ce principe est de laisser ce noeud "cerveau" s'occuper
des stratégies a suivre, avec un niveau plus haut d'abstraction.

## Interfaces
### Serveur d'action
Ce noeud met un [serveur d'action](https://design.ros2.org/articles/actions.html)
à disposition des autres noeuds afin de pouvoir communiquer.

Dans un premier temps, un client doit être créé et se connecter au serveur d'action.
Dans le cas de l'action sequencer le nom du serveur est `/<namespace>/actionsequencer`,
avec `<namespace>` correspondant au namespace dans lequel est démarré le noeud.
Si aucun namespace n'à été déclaré, le serveur sera accessible sous : `/actionsequencer`.

L'interêt d'utiliser un serveur d'action est de pouvoir gérer facilement l'exectution
de différentes tâches, à travers son API comme:

- Envoyer le nom de la tâche à être executée avec d'autres paramètres
  complémentaires optionels.
- Suivre l'avancement de cette tâche grâce à des messages de Feedbacks
  fournis par le serveur.
- Demander l'annulation d'un tâche particulière.
- Récupérer le résultat d'une action terminée, annulée par le client ou par le
  serveur.

#### Action Messages
##### Goal
Afin de confier une nouvelle tâche à l'action sequencer, un client doit lui envoyer un `Goal`
Ce `Goal`, se matérialise ici sous la forme d'un message particulier ayant la structure suivante:
```c
std_msgs/String order
std_msgs/String argv
```
Le champs `order` défini le nom de la tâche à executer, se matérialisant par une des séquences d'actions
définies dans un fichier de configuration (cf [ICI](#définition-d’une-séquence-d’action)).
Le champs `argv` correspond lui à une chaine de caractères dont chaque éléments est séparé par des espace.


> [!NOTE]
> L'action sequencer peut détecter si des paramètres nécéssaires à l'execution sont manquants ou bien si
> le nombre de paramètres fournis est trop important.

##### Feedback
Lorsque une tâche est en cours d'execution, le serveur d'action retourne un Message de type `Feedback`.
Ce message contient une liste de toutes les actions qui sont actuellements executées par la séquence d'action.
Un gros volume de message peut être échangé sur ce topic puisqu'à chaque fois qu'une action est lancée ou bien terminée,
un nouveau message de `Feedback` est communiqué.
La structure de ce message est la suivante :
```c
std_msgs/String current_actions
```

##### Result
Plusieurs cas peuvent mener à l'envoi d'un résultat.
Tout d'abord un résultat peut être envoyé lorsqu'une tâche vient d'être accomplie.

Ensuite, cela peut être suite à une annulation d'un tâche, de la part de l'utilisateur.

Et pour finir, dans le pire des cas, suite à un problème d'execution, qui rend la reprise
de la séquence d'actions impossible.

Pour tous les cas ci-dessus, ce résultat est une simple chaine de caractère de la forme suivante :
```c
std_msgs/String context
```
Cependant, pour tout de même pouvoir différencier une tâche accomplie d'une tâche annulée ou bien
d'un problème d'execution, la bibliothèque `ros_action_client` permet de récupérer
un status, spécifiant ce cas (plus d'infos [ICI](https://docs.ros2.org/latest/api/rclpy/api/actions.html#module-rclpy.action.client)
[cdf_msgs](https://github.com/7Robot/cdf_msgs).

### Subscribers et Publishers
Les subscribers et publishers exposés par ce noeuds ne peuvent pas être connus
par avance.
En effet, comme ce sont les séquences d'actions qui définissent leurs besoins à
ce niveau, les Subscribers et Publishers utilisés par le noeud seront donc
variables.

### Paramètres
| Nom | Type | Valeur Par Défaut | Description |
| --- | ---- | ----------------- | ----------- |
| `action_sequence_file`| str | `config/action_sequence_default.py` | Définit le fichier contenant les séquences d'actions pouvant être utilisées |
| `team_color` | str | `None` | Définit la couleur de l'équipe pour lequel le robot va jouer |

## Définition d'une séquence d'action
Chaque séquence d'actions doit hériter de la classe `ActionSequence` et être
placée dans un fichier, qui contiendra toutes les autres séquences d'actions
pouvant être appelées.

### Attributs obligatoires
Deux attributs doivent êtres définis pour chaque nouvelle séquences d'actions
crées :
- `subscribers`: Liste de tuples de la forme : `("<topic_path>", topic_type, callback_func)`
    - topic_path: Nom du topic sur lequel écouter
    - topic_type: Type des messages echangés sur ce topic
    - callback_func: Fonction appelée lorsqu'un nouveau message est recu sur le
      topic.
- `publishers`: Liste de tuple de la forme `("<topic_path>", topic_type`)
    - topic_path: Nom du topic sur lequel écouter
    - topic_type: Type des messages echangés sur ce topic

> [!IMPORTANT]
> La définition des bons subscribers et publishers à utiliser, ainsi que le choix
> des callbacks est la facette la plus compliquée de la définition
> d'une séquence d'action.
> Elle nécéssite une compréhension des actions utilisées, et de leurs
> différents besoins en topics.

### Définition de la fonction `run()`
L'ensemble des actions constituant une séquence d'actions sont définis lors de 
l'appel de la fonction `run()`.
Cette fonction `run()` peut être peuplée par diverse méthodes permettant de
lancer des actions ainsi que de contrôler leur execution.

Lorsque l'on définit la fonction `run()`, il est impératif de faire apparaitre
les paramètres `self` ainsi que les paramètres attendus si il y en a.
En effet, le paramètre `self` permet d'acceder au méthodes propres à la classe
qui servent par exemple a déclarer de nouvelles actions à executer.

> [!NOTE]
> Il est important de noter que étant donné que l'execution de la séquence se fait
> via l'appel d'une fonction, des éléments logiques peuvent être incorporés.
> Cette logique peut être utile lorsque l'on souhaite modifier l'execution de la séquence
> en fonction du contexte dans laquelle elle se trouve.

#### `add_action(func, arg_1, arg_2, ... , arg_n, blocking=False)`
Permet d'ajouter une action à la séquence. Cette action est lancée par défaut
de manière asynchrone, et est donc non-bloquante. Cependant si l'on souhaite
que l'action soit terminée avant de passer à la suite, on peut définir le
paramètre `blocking` à `True`.

Cette methode retourne un objet de type `Action` qui en réalité hérite de la
classe `threading.Thread`. Cet objet peut être utilisé afin de récupérer un
résultat retourné par la fonction associée à l'action. Ou bien a définir des
dépendances concernant le flux d'execution.

#### `wait_for(action_1, action_2, ... , action_n)`
Cette methode permet de contrôler le flux d'execution des actions de la
séquence, en attendant que les actions listées soit terminées.

Cette fonction peut être utile lorsque certaines actions on des
dépendances, ou bien que les résultats fournis par une action précédente sont
requis pour passer à la suite de la séquence.

### Définition de la fonction `cancel()`
Cette fonction n'est pas obligatoire dans la définition de la séquence
d'action.
Elle permet de définir la facon de gérer la manière dont une séquence
est interrompue en définissant de possible actions complémentaires à
effectuer après une annulation ou un problème d'execution.
l'execution d'actions complémentaires.

Par défaut, lorsque l'on ne re-défini pas cette fonction, la procédure est de
stopper toutes les actions actuellement en cours d'execution, en commencant par
l'action la plus ancienne.
