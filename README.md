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

### Key Functions

-   **`handleEvent(Event_t event)`**:  
    This function manages the state transitions of the robot based on the received event. The function evaluates the current state of the robot and the new event to decide which state the robot should transition to next. For instance, receiving a forward event will transition the robot to a forward movement state, while receiving a stop event will lead to a neutral state. Each state corresponds to a different action, such as moving forward, rotating, or stopping.
    
-   **`executeStateActions()`**:  
    This function executes the actions associated with the current state of the robot. Depending on the state, it adjusts the speed and direction of the motors using PWM signals and GPIO pins. For example, if the robot is in a forward state, this function sets the PWM signal to drive the motors forward. If in reverse, it sets the PWM signals accordingly and reverses the motor direction.
    
-   **`calculCommande(int mode, int cote)`**:  
    This function calculates the control command for the motors based on the desired speed (mode) and the side of the robot (left or right). It uses a Proportional Integral (PI) controller to adjust the motor speed, taking into account the error between the target speed and the actual speed, as measured by the encoders. The function adjusts the motor speed by calculating the required PWM signal to achieve the desired speed.
    
-   **`HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)`**:  
    This function is called each time the timer period elapses. It is primarily used to increment time-based counters like `T_batt` and `T_enc` that control periodic tasks, such as battery level monitoring and encoder value reading.
    
-   **`HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)`**:  
    This function is called when the ADC (Analog to Digital Converter) finishes converting the battery voltage. It stores the conversion result and triggers further action, such as alerting the user if the battery voltage is below a critical level.
    
-   **`HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)`**:  
    This function handles incoming data over the Bluetooth connection. When a complete command is received (e.g., forward, backward, left, or right), it sets the corresponding event to trigger a state change in the robot.

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

### Fonctions Clés

-   **`handleEvent(Event_t event)`** :  
    Cette fonction gère les transitions d'état du robot en fonction de l'événement reçu. La fonction évalue l'état actuel du robot et le nouvel événement pour déterminer l'état vers lequel le robot doit se diriger ensuite. Par exemple, la réception d'un événement "avancer" fera passer le robot à un état de mouvement en avant, tandis que la réception d'un événement "arrêt" ramènera le robot à l'état neutre. Chaque état correspond à une action différente, comme avancer, tourner ou s'arrêter.
    
-   **`executeStateActions()`** :  
    Cette fonction exécute les actions associées à l'état actuel du robot. Selon l'état, elle ajuste la vitesse et la direction des moteurs en utilisant des signaux PWM et des broches GPIO. Par exemple, si le robot est dans un état d'avance, cette fonction configure le signal PWM pour faire avancer les moteurs. Si le robot est en marche arrière, elle ajuste les signaux PWM en conséquence et inverse la direction des moteurs.
    
-   **`calculCommande(int mode, int cote)`** :  
    Cette fonction calcule la commande de contrôle pour les moteurs en fonction de la vitesse souhaitée (mode) et du côté du robot (gauche ou droite). Elle utilise un contrôleur Proportionnel Intégral (PI) pour ajuster la vitesse des moteurs en tenant compte de l'erreur entre la vitesse cible et la vitesse réelle mesurée par les encodeurs. La fonction ajuste la vitesse des moteurs en calculant le signal PWM nécessaire pour atteindre la vitesse souhaitée.
    
-   **`HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)`** :  
    Cette fonction est appelée chaque fois que la période du timer s'écoule. Elle est principalement utilisée pour incrémenter des compteurs liés au temps comme `T_batt` et `T_enc` qui contrôlent les tâches périodiques, telles que la surveillance de la batterie et la lecture des valeurs des encodeurs.
    
-   **`HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)`** :  
    Cette fonction est appelée lorsque le convertisseur analogique-numérique (ADC) termine la conversion de la tension de la batterie. Elle stocke le résultat de la conversion et déclenche une action supplémentaire, comme alerter l'utilisateur si la tension de la batterie est inférieure à un niveau critique.
    
-   **`HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)`** :  
    Cette fonction gère les données entrantes via la connexion Bluetooth. Lorsqu'une commande complète est reçue (par exemple, avancer, reculer, gauche ou droite), elle définit l'événement correspondant pour déclencher un changement d'état du robot.
## Auteurs

Projet scolaire pour l'École des Mines de Saint-Étienne.  
[Martin RABIER](https://github.com/MartinRabier) @MartinRabier  
[Tristan Panhelleux](https://github.com/tristanplx) @tristanplx

![Alt text](https://github.com/user-attachments/assets/a80c120f-dad3-47e3-9ee0-82401f6e7e81)

