Controlador cartesiano de trayectorias que lee el aruco (normal):
aruco_trajectory_cartesian_controller.yaml --> aquí puedes cambiar los parámetros que necesites
- Para lanzarlo:
- T1: roslaunch astronaut astronaut.launch
- T2: roslaunch astronaut trajectory_motion.launch
Controlador que tiene en cuenta el efecto de los pares en la base:
torso_effort_controller.yaml --> aquí puedes cambiar los parámetros que necesites
- Para lanzarlo:
- T1: roslaunch astronaut astronaut.launch
- T2: roslaunch astronaut torso_effort_motion.launch
Controlador de trayectorias basic que recibe una posición por un topic (basic):
cartesian_controller_basic.yaml --> aquí puedes cambiar los parámetros que necesites (carpeta config de astronaut_controllers)
cartesian_frame_topic_example --> aquí está el ejemplo de punto que te he puesto en la terminal 3 (carpeta INFO de astronaut_controllers)
- Para lanzarlo:
(Es más cómodo probarlo en un mundo vacío pero funciona en cualquier mundo claro)
- T1: roslaunch astronaut astronaut.launch testing:=true
- T2: roslaunch astronaut_controllers cartesian_controller_basic.launch
- T3: rostopic pub -r 10 /cartesian_target_frame astronaut_controllers/target_frame "{x: 0.30, y: -0.25, z: 0.0, roll: 0.0, pitch: 0.0, yaw: -3.14, duration: 10.0}"
NUEVO BASIC
- Para lanzar el robot en el mundo sin la ISS
T1 : roslaunch astronaut astronaut.launch testing:=true (hands)
o
T1: roslaunch astronaut astronaut_gripper.launch testing:=true (grippers)
- Para lanzar el controlador
T2 : roslaunch astronaut_controllers cartesian_controller_torso_efforts.launch
- Para enviar el topic:
(Para hands)
T3 : rostopic pub -r 10 /cartesian_target_frame astronaut_controllers/target_frame "{x: 0.45, y: -0.25, z: 0.0, roll: 3.14, pitch: 1.57, yaw: 0.0, duration: 9.0}"
o
(Para Grippers)
T3: rostopic pub -r 10 /cartesian_target_fra astronaut_controllers/target_frame "{x: 0.45, y: -0.25, z: 0.0, roll: 0.0, pitch: 1.57, yaw: 3.14, duration: 9.0}"
rosbag record -O bag_name /control_error /final_error /velocity_error /joint_value /frames_trajectory
