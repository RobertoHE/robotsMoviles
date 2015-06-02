El proyecto está pensado para utilizarlo sobre una RaspberryPi y un Arduino para controlar los servomotores con una señal de 5V

Fast Marching basado en el proyecto de Javier V. Gomez
Se necesita tener instaladas las librerias OpenCV
Se necesita tener instaladas las librerias CMAKE
Se necesita tener las fuentes del proyecto UUGear (Utiliza Arduino como un periférico esclavo)

Uso:
Cargar el archivo .ino del proyecto UUGear en una placa Arduino

./lsuu    			(Muestra el nombre que tiene la placa Arduino)

Cambiar el nombre de la placa en el archivo main.cpp en la linea 68

mkdir build && cd build && cmake .. && make

Para ejecutar desde la carpeta build:
./fmm -map2 ../data/ceabotX.png      (se puede cambiar la imagen del mapa)




Si se usan fuera de una rpi o startX activado, se puede descomentar las funciones  "imshow" y "GridPlotter" para que muestre los procesos por pantalla
