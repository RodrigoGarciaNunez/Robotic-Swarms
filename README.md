# ProyectoTerminal_AlgoritmosNeuronasEspejoParaAprendizajeDeEnjambresRoboticos

## Acerca de la implementacion

Este proyecto utiliza ROS2 para implementar los elementos de la arquitectura cognitiva, así como sus comunicaciones.

Para este poryecto, se han creado 2 paquetes de ros2: arlo_interfaces y cognitive_architecture.

## Paquete "cognitive_architecture"

Este es el paquete principal del proyecto. Es un paquete que contiene scripts de python y códigos fuente en c++. 
El paquete está dividido en los siguientes Directorios:

- cognitive_architecture
  -  cognitive_architecture
  -  include
  -  launch
  -  models
  -  scripts
  -  src
  -  test
  -  worlds
  -  CMakeLists.txt
  -  package.xml
    
### Directorio cognitive_architecture
Este directorio cuenta con los scripts python de los nodos "corteza_motora_primaria" y "lobulo_temporal"

### Directorio include
Este directorio cuenta con los códigos fuente en c++ de los nodos "corteza_motora_secundaria" y "corteza_premotora", así como códigos extra para su funcionamiento (consultar README del directorio)

### Directorio launch
Este directorio contiene los archivos de lanzamiento para la simulación, con configuraciones que van desde el mundo a iniciar y los modelos que aparecerán en este.

### Directorio models
Cuenta con los archivos sdf de los modelos a utilizar en la simulación

### Directorio scripts
Contine los scripts utilizados para inicializar los nodos escritos en python

### Directorio src
Contiene los códigos fuente que inicializan los nodos escritos en c++

### Directorio test
Carpeta test predeterminada de ros2

### Directorio worlds
COntinene los archivos xml que definen los mundos que se pueden lanzar en la simulación de gazebo.

### CMakeLists.txt
Este archivo se encarga del control de la compilación del paquete "cognitive_architecture".

### package.xml
Contiene todas las dependencias del paquete.


## Paquete "arlo_interfaces"
En este paquete se encuentran los mensajes y servicios personalizados.
El paquete está dividido en los siguientes Directorios:

-  arlo_interfaces
  -  msg
  -  srv
  -  CMakeLists.txt
  -  package.xml

###Directorio msg
Contiene los archivos que definen los mensajes personalizados para el proyecto

### DIrectorio srv
Contiene los archivos que definen los servicios personalizados para el proyecto

### CMakeLists.txt
Este archivo se encarga del control de la compilación del paquete "arlo_interfaces".

### package.xml
Contiene todas las dependencias del paquete.


