# turtlebot_easyprog
Einfache Skripte zur Verhaltensprogrammierung eines Turtlebot(2) für Schüler. Zum Ausprobieren ist KEIN realer Turtlebot 2 notwendig. Viele Experimente lassen sich auch im Simulator durchführen. 

## Bestandteile
Dieses Repository enthält die Bestandteile:
* `turtlebot_easyprog` Softwarebibliothek in [Python](https://www.python.org/) zur abstrahierten Programmierung eines ROS Roboters wie bspw. Turtlebot 2
* `Turtle-Dance` GUI-Anwendung zur grafischen Erstellung von Choreographie der Roboterbewegung. Ausführung in Simulator und real möglich.

## Installationshinweise
Es wird ein auf [Ubunutu 14.04](https://wiki.ubuntuusers.de/Trusty_Tahr/) basierendes Linux-Betriebssystem empfohlen.
* Installation von [ROS Indigo](http://wiki.ros.org/indigo) (Desktop-full)
* Einrichten eines catkin workspace `cd ~ ; mkdir -p catkin_ws/src; cd cakin_ws/src; catkin_init_workspace; `
* Clone des Repostories nach `~/catkin_ws/src/turtlebot_easyprog` (andere Pfade werden derzeit nicht unterstützt!)
* Catkin Make `cd ~/catkin_ws; catkin_make;`
* OPTIONAL: Hinzufügen von Shortcuts zur Bash-Umgebung `source ~/catkin_ws/src/turtlebot_easyprog/setup.bash` bspw. in `.bashrc` sowie `export MASTER=IP-Adresse ROS-Master` und `export MYIP=IP-Adresse Steuerrechner`
