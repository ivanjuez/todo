#!/usr/bin/env python
# coding=utf-8

#Defino todo lo que necesito de estas librerias
import sys
import rospy
import copy, math
from math import pi 
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import moveit_msgs.msg
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
from copy import deepcopy

#Defino el nombre del robot
ROBOT_NAME = "irb120_robotiq85"


if ROBOT_NAME == "irb120_robotiq85":
#Defino los diferentes grupos de MOVEGROUP como son el brazo por un lado y la pinza
    GROUP_NAME_ARM = 'irb_120'
    GROUP_NAME_GRIPPER = 'robotiq_85'

    GRIPPER_FRAME = 'gripper_base_link' #mi duda es si aqui debo poner donde se fija el dedo "gripper_finger1_joint" o directamente sería el mismo

    FIXED_FRAME = 'base_link'
#Defino las posiciones de la pinza en función de la posición del joint finger 1
    GRIPPER_CLOSED = [0.02]
    GRIPPER_OPEN = [0.0]
    GRIPPER_NEUTRAL = [0.03]
#En este caso defino el joint que se va a mover de la pinza 
    GRIPPER_JOINT_NAMES = ['gripper_finger1_joint']

    #GRASP_OVERTIGHTEN = 0.002
    GRASP_OVERTIGHTEN = 0.002
    GRIPPER_EFFORT = [1.0]

    max_pick_attempts = 5
###################################################	
### FUNCIONES QUE IRE LLAMANDO CUANDO NECESITE  
###################################################


# IK ROBOT: Funcion para mover el TCP a una posicion y orientacion determinada
def move_pose_arm(roll,pitch,yaw,x,y,z):
    pose_goal = geometry_msgs.msg.Pose()
    quat = quaternion_from_euler(roll,pitch,yaw)
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    arm.set_pose_target(pose_goal)

    plan = arm.go(wait=True)

    arm.stop()
    arm.clear_pose_targets()

# Funcion para mover la herramienta 
def move_joint_gripper(joint):
    joint_goal = gripper.get_current_joint_values()
    joint_goal[0] = joint
    joint_goal[1] = joint

    gripper.go(joint_goal, wait=True)
    gripper.stop() # Garantiza que no hay movimiento residual

    
# Defino la postura de la pinza como JointTrajectory
def make_gripper_posture(joint_positions):
	#inicilizo el joint trayectory para los joint de la pinza
        t = JointTrajectory()
	#Defino los nombres de los joint de la pinza, en mi caso no se si solo se pone el 1 o los 2
        t.joint_names = GRIPPER_JOINT_NAMES
	#Asigno la una trayectoria de punto que represente el objetivo
        tp = JointTrajectoryPoint()
	#Asigno la trayectoria a los joints
        tp.positions = joint_positions
	#FIjo el gripper effort
        tp.effort = GRIPPER_EFFORT
	tp.time_from_start = rospy.Duration(1.0)
	#adjunto los puntos a la trayectoria de puntos
        t.points.append(tp)
        return t

    #Ahora creo la translacion de la distancia del objeto a coger , sería como la pre_grap_approach y el post_grasp_retreat, nos devuelve la dirección del vector y la distancia deseada y la distancia mínima al objeto
def make_gripper_translation(min_dist, desired, vector):
	#Inicio el objeto grippert translation
        g = GripperTranslation()
	 # Defino las componentes del vector
	g.direction.vector.x = vector[0]
	g.direction.vector.y = vector[1]
	g.direction.vector.z = vector[2]
	#EL vector es relativo al gripper_frame
        g.direction.header.frame_id = GRIPPER_FRAME
	#Defino la distancia minima y deseada al objetivo
        g.min_distance = min_dist
        g.desired_distance = desired
        return g

    # Genero una lista de posibles grasps
def make_grasps(initial_pose_stamped, allowed_touch_objects, grasp_opening=[0]):
	# INicio el grasp
	g = Grasp()

	# Asigno las posiciones de pre-grasp y grasp postures;
	
	g.pre_grasp_posture = make_gripper_posture(GRIPPER_OPEN)
	g.grasp_posture = make_gripper_posture(grasp_opening)

	# Defino los parametros de aproximación y alejar deseados
	g.pre_grasp_approach = make_gripper_translation(0.01, 0.1, [1.0, 0.0, 0.0])
	g.post_grasp_retreat = make_gripper_translation(0.1, 0.15, [0.0, -1.0, 1.0])

	# Defino la primera grasp pose
	g.grasp_pose = initial_pose_stamped

	# Pitch angulos a probar: POR AHORA SOLO UNO
	pitch_vals = [0]

	# Yaw angles to try: POR AHORA SOLO UNO
	yaw_vals = [0]
	
	# A list to hold the grasps
	grasps = []

	# Genero un grasp para cada angulo pitch y yaw
	for y in yaw_vals:
	    for p in pitch_vals:
		# Creo un quaternion de Euler angles, con un roll de pi (cenital)
		q = quaternion_from_euler(pi, p, y)

		# asigno grasppose acorde al quaternio
		g.grasp_pose.pose.orientation.x = q[0]
		g.grasp_pose.pose.orientation.y = q[1]
		g.grasp_pose.pose.orientation.z = q[2]
		g.grasp_pose.pose.orientation.w = q[3]

		# EStablezco la id de este grasp 
		g.id = str(len(grasps))

		# Defino los objetos que se pueden tocar
		g.allowed_touch_objects = allowed_touch_objects
	
		# no elimino la fuerza de contacto
		g.max_contact_force = 0
 
	
		g.grasp_quality = 1.0 - abs(p)

		# Adjunto la lista de grasp
		grasps.append(deepcopy(g))
	
	#Me devuelve la lista
	return grasps

 
if __name__=='__main__':

	##################################################################################################
	# BLOQUE 1: Stuff inicial
	##################################################################################################
	#INicio el nodo obejtivo1y2 de rospy
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('objetivo1y2')
      	#Lanzo la escena y el robot
        scene = moveit_commander.PlanningSceneInterface()
        robot = moveit_commander.RobotCommander()
	# Create a scene publisher to push changes to the scene
	scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)
	# Create a publisher for displaying gripper poses
	gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=10)
       	#INicializo el move group del brazo y la pinza
        arm = moveit_commander.MoveGroupCommander(GROUP_NAME_ARM)
        gripper = moveit_commander.MoveGroupCommander(GROUP_NAME_GRIPPER)    	
	#CON ESTO obtenemos EL link END-EFFECTOR DEL BRAZO
        eef = arm.get_end_effector_link()	
	arm.allow_replanning(True)
	

	# Para publicar trayectorias en RViz
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)


	# Debug Diego
	print "\n El End effector del brazo detectado es= ", eef
	
        # Estado del robot
        print "\n ============ Printing robot state ============"
        print robot.get_current_state()
        print ""  
	
	# Asigno el frame de referencia
	arm.set_pose_reference_frame(FIXED_FRAME)	
	# Doy 5 segundos para cad aintento
	arm.set_planning_time(5)
	# POngo el limite de numero de intentos
	max_pick_attempts = 1

	# CREO CAJA
	box1_id='box1'	
	#DEfino las dimensiones de la caja
	box1_size=[0.05, 0.05, 0.05]
	# LImpiamos la caja (si existe de ejecuciones previas)
        scene.remove_world_object(box1_id)

	# MUEVO ROBOT A HOME. ABRO PINZA
        rospy.loginfo("Moving arm to HOME")	
	move_pose_arm(pi,0,-pi/4,0.5,-0.1,0.7)
	#move_pose_arm(0,0,0,0,0,0)
        rospy.sleep(2)
        rospy.loginfo("Opening gripper")
        move_joint_gripper(0.0)

	raw_input("\n Robot listo. BLOQUE 1 terminado. Press key to continue")

	##################################################################################################
	# BLOQUE 2: Creacion escena y poses
	##################################################################################################
       	

	#Añado la caja a la escena
        box1_pose = PoseStamped()
        box1_pose.header.frame_id = robot.get_planning_frame()
   
        # Añadimos la caja que será cogida (alejada por el momento)
        box1_pose.pose.position.x = 0.5
        box1_pose.pose.position.y = 0.3
        box1_pose.pose.position.z = 0.3
        scene.add_box(box1_id, box1_pose, box1_size)
        rospy.sleep(1)
        
	print "\n La pose del pick creada es= \n", box1_pose
	raw_input("\n BLOQUE 2 terminado. Press key to continue")

	##################################################################################################
	# BLOQUE 3: Grasping con pick()
	##################################################################################################

	#Inicializo el grasp definiendo el grasp pose con la ubicación de mi caja
	grasp_pose = box1_pose

	# FIjo la grasp pose un poco más arriba por ahora, para evitar colision pinza caja
	grasp_pose.pose.position.z = grasp_pose.pose.position.z 

	# Genero una lista de grasps
	grasps = make_grasps(grasp_pose, [box1_id], [box1_size[1] - GRASP_OVERTIGHTEN])

	# Publico las grasp poses para verlas en RVIZ
	for grasp in grasps:
	    gripper_pose_pub.publish(grasp.grasp_pose)
	    print "\n ------------------------------------------------------------------\n"
	    print "\n grasp =\n",grasp #debug 
	    rospy.sleep(0.2)
	# Pruebo exito/fallo y el numero de intentos para la operación de pick
	result = MoveItErrorCodes.FAILURE
	n_attempts = 0

	# PICK: Repito hasta que tenemos un buen resultado 
	while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
	    result = arm.pick(box1_id, grasps)
	    n_attempts += 1
	    rospy.loginfo("Pick intentos: " + str(n_attempts))
	    rospy.sleep(0.2)

	# Si el pick es exitoso, intento la locacilación del pick
	if result == MoveItErrorCodes.SUCCESS:
	    result = arm.pick(box1_id, grasps)
	    success = False
	    n_attempts = 0
	    
	else:
	    rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")
	
	# MUEVO ROBOT CON OBJETO COGIDO
        rospy.loginfo("Moving arm while box is picked")	
	move_pose_arm(0,0,0,0,0,0)
        rospy.sleep(4)

	# ELimino cualquier objeto cogido
	scene.remove_attached_object(GRIPPER_FRAME, box1_id)
        rospy.sleep(1)	
        # LImpiamos la caja a coger
        scene.remove_world_object(box1_id)
        rospy.sleep(1)	

	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)

