[English <img src="https://cdn-icons-png.flaticon.com/16/197/197374.png" width="16"/>](#english-version)  
[Français <img src="https://cdn-icons-png.flaticon.com/16/197/197560.png" width="16"/>](#version-fran%C3%A7aise)

# ProjetRobot

<a name="english-version"></a> English Version

## Robot Control Project

This project implements the control of a robot via a Bluetooth-enabled mobile application. The robot can move forward, backward, and rotate. The speed is controlled using Pulse Width Modulation (PWM), and the battery level is monitored. A key part of this project involves the implementation of a state machine to handle the robot's different modes and movements.

### State Machine Design

The robot's movement and behavior are managed using a state machine, which allows for smooth transitions between different states (e.g., moving forward, backward, or rotating). Events triggered by the Bluetooth commands (such as forward, backward, left, and right) are handled by the state machine, ensuring the robot behaves according to the user's input.

Each state corresponds to a movement (e.g., moving forward at different speeds, rotating, or stopping), and transitions between states occur based on the commands received via Bluetooth. The robot also transitions through various speed levels, with each mode having precise speed control through PWM.

### PWM Speed Control

Speed control is achieved using PWM signals. Depending on the state and user commands, the PWM duty cycle is adjusted to control the motor speed with three distinct speed levels (slow, medium, and fast).

### Proportional Integral Control (PI) for Motor Control

In this project, a Proportional Integral (PI) control system was implemented to ensure precise speed regulation. The robot's motors are equipped with encoders that provide real-time feedback on the motor speed. Using this feedback, the PI controller adjusts the motor speed to match the desired setpoint, compensating for any discrepancies.

### Battery Monitoring

The robot's battery level is monitored through ADC (Analog to Digital Converter). If the battery voltage drops below a critical level, an LED alerts the user, allowing the robot to continue functioning safely.

### Key Features

-   **Robot Movement**: Forward, backward, and rotation commands with adjustable speed.
-   **Bluetooth Control**: The robot is controlled via Bluetooth, with commands being processed through a state machine.
-   **PWM Control**: Precise speed control using PWM for both forward and reverse directions.
-   **Battery Monitoring**: A low battery warning system using an ADC to monitor the voltage level.
-   **PI Control**: Speed regulation using Proportional Integral control for consistent motor performance.

## Authors

School project for the Mines of Saint-Etienne.  
[Martin RABIER](https://github.com/MartinRabier) @MartinRabier  
[Tristan Panhelleux](https://github.com/tristanplx) @tristanplx

----------

<a name="version-française"></a> Version Française

## Projet de Contrôle de Robot

Ce projet implémente le contrôle d’un robot via une application mobile utilisant la communication Bluetooth. Le robot peut se déplacer en avant, en arrière, et tourner. La vitesse est contrôlée via la modulation de largeur d’impulsion (PWM), et le niveau de batterie est surveillé. Un aspect clé de ce projet est l’implémentation d’une machine d’état pour gérer les différents modes et déplacements du robot.

### Conception de la Machine d’État

Les déplacements et comportements du robot sont gérés par une machine d’état, ce qui permet des transitions fluides entre les différents états (par exemple, avancer, reculer ou tourner). Les événements déclenchés par les commandes Bluetooth (comme avancer, reculer, tourner à gauche ou à droite) sont traités par la machine d’état, garantissant que le robot réagit selon les entrées de l’utilisateur.

Chaque état correspond à un mouvement (par exemple, avancer à différentes vitesses, tourner ou s’arrêter), et les transitions entre les états se font en fonction des commandes reçues via Bluetooth. Le robot passe également par plusieurs niveaux de vitesse, chaque mode ayant un contrôle précis de la vitesse via la PWM.

### Contrôle de Vitesse par PWM

La gestion de la vitesse est effectuée à l'aide de signaux PWM. En fonction de l'état et des commandes de l'utilisateur, le rapport cyclique PWM est ajusté pour contrôler la vitesse des moteurs avec trois niveaux distincts (lent, moyen, et rapide).

### Asservissement Proportionnel Intégral (PI) pour le Contrôle des Moteurs

Dans ce projet, un système de contrôle proportionnel intégral (PI) a été mis en place pour garantir une régulation précise de la vitesse. Les moteurs du robot sont équipés d'encodeurs qui fournissent un retour d'information en temps réel sur la vitesse des moteurs. Grâce à ce retour, le contrôleur PI ajuste la vitesse des moteurs pour correspondre à la consigne souhaitée, compensant ainsi les écarts.

### Surveillance de la Batterie

Le niveau de la batterie du robot est surveillé via un convertisseur analogique-numérique (ADC). Si la tension de la batterie descend en dessous d’un seuil critique, une LED alerte l’utilisateur, permettant au robot de continuer à fonctionner en toute sécurité.

### Fonctionnalités Clés

-   **Déplacement du Robot** : Commandes d’avancer, reculer et tourner avec vitesse ajustable.
-   **Contrôle via Bluetooth** : Le robot est contrôlé via Bluetooth, les commandes étant traitées par une machine d’état.
-   **Contrôle de la Vitesse par PWM** : Contrôle précis de la vitesse avec PWM pour les directions avant et arrière.
-   **Surveillance de la Batterie** : Un système d’alerte en cas de batterie faible utilisant un ADC pour surveiller le niveau de tension.
-   **Asservissement PI** : Régulation de la vitesse des moteurs grâce à un contrôle proportionnel intégral.

## Auteurs

Projet scolaire pour l'École des Mines de Saint-Étienne.  
[Martin RABIER](https://github.com/MartinRabier) @MartinRabier  
[Tristan Panhelleux](https://github.com/tristanplx) @tristanplx
