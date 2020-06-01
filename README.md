# todo
Publico todo lo que tengo dentro de la carpeta SRC.
He modificado los archivos denominandolos como _final para los puedas ver. Esos ficheros modificados son:

·Dentro del directorio de ABB_experimental:

  ·abb_irb120_gazebo/urdf : aquí he modificado los ficheros irb120_3_58.xacro y irb120_3_58_macro.xacro.
  ·abb_irb120_support/urdf : aquí he modificado los ficheros irb120_3_58.xacro y irb120_3_58_macro.xacro.
  
·Dentro del directorio de irb120_robotiq85:

  ·irb120_robotiq85_moveit_config/config : aquí he modificado el fichero irb120_robotiq85.srdf
  
  ·irb120_robotiq85_gazebo/urdf : aquí he modificado el fichero irb120_robotiq85_macro.xacro
  
  ·irb120_robotiq85_gazebo/launch : aquí he modificado el fichero irb120_robotiq85_gazebo_moveit_rviz.launch
  
 De esta forma, ya he conseguido que me lanze correctamente el brazo y la pinza.
 
 Para lanzarlo utilizo el siguiente comando:
 
 ·roslaunch irb120_robotiq85_gazebo irb120_robotiq85_gazebo_moveit_rviz_final.launch
 
 Para ejecutar el código para realizar el pick es:
 
 ·rosrun pick_place objetivo1y2.py


 
 
