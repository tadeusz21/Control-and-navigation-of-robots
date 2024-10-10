Zadanie 1
Po użyciu polecenia zaimportowane zostały do przestrzeni roboczej do katalogu src pakiety pozwalające na obsługę robota tiago.

Zadanie2 
Po użyciu polecenia source zostało przygotowane środowusko do pracy z ros2 iron 
Natomoiast p uzyciu colcon build --symlink-install pliki w przestrzeni roboczej zostały zbudowane. Użycie flagi --symlink-install usprawnia proces poprzez modyfikowanie j8uż zainstalowanych pliuków zamiast tworzynia nowych.

Zadanie 3
Po uruchoimieniu programu w terminalu zostało wyświetlne "hello world hello_stero package"

Zadanie 4 
Węzeł /move_group udostępnia akcję, natomaist /play_motion2_move_group_node z niej korzysta.

Zadanie 5 
Po dodaniu wizualizacji pojawił się nowy klient /rviz. 

Zadanie 6 
Serwisy udestępniane przez /move_group:
    /apply_planning_scene: moveit_msgs/srv/ApplyPlanningScene
    /check_state_validity: moveit_msgs/srv/GetStateValidity
    /clear_octomap: std_srvs/srv/Empty
    /compute_cartesian_path: moveit_msgs/srv/GetCartesianPath
    /compute_fk: moveit_msgs/srv/GetPositionFK
    /compute_ik: moveit_msgs/srv/GetPositionIK
    /get_planner_params: moveit_msgs/srv/GetPlannerParams
    /load_map: moveit_msgs/srv/LoadMap
    /move_group/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /move_group/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /move_group/get_parameters: rcl_interfaces/srv/GetParameters
    /move_group/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /move_group/list_parameters: rcl_interfaces/srv/ListParameters
    /move_group/set_parameters: rcl_interfaces/srv/SetParameters
    /move_group/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /plan_kinematic_path: moveit_msgs/srv/GetMotionPlan
    /query_planner_interface: moveit_msgs/srv/QueryPlannerInterfaces
    /save_map: moveit_msgs/srv/SaveMap
    /set_planner_params: moveit_msgs/srv/SetPlannerParams

Akcje udostępniane przez /move_group:
    /execute_trajectory: moveit_msgs/action/ExecuteTrajectory
    /move_action: moveit_msgs/action/MoveGroup


Serwisy udestępniane przez /move_group_private_108906206361280:
    /get_planning_scene: moveit_msgs/srv/GetPlanningScene
    /move_group_private_108906206361280/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /move_group_private_108906206361280/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /move_group_private_108906206361280/get_parameters: rcl_interfaces/srv/GetParameters
    /move_group_private_108906206361280/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /move_group_private_108906206361280/list_parameters: rcl_interfaces/srv/ListParameters
    /move_group_private_108906206361280/set_parameters: rcl_interfaces/srv/SetParameters
    /move_group_private_108906206361280/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically

W tym przypadku nie są udostępniane żadne akcje.

Węzeł move_group służy do planowania trasy i zlecania polecenia ruchu. Natomaist


Zadanie 7
Węzeł /moveit_simple_contraoller_manager nadzoruje generownae trajektorie poszczególnych części robota.

Zadanie 8
Dla złącza ramienia 1 dostępne są nastœpujące comand_interfaces:
    arm_1_joint/effort 
	arm_1_joint/position 
	arm_1_joint/velocity

Używany jest: arm_1_joint/position 

Interfacey stanu:
    arm_1_joint/effort
	arm_1_joint/position
	arm_1_joint/velocity


Zadanie 9

state_interfaces:state_start_arm_1_joint/effort -> joint_state_broadcaster:state_end_arm_1_joint/effort
state_interfaces:state_start_arm_1_joint/velocity -> joint_state_broadcaster:state_end_arm_1_joint/velocity
arm_controller:command_start_arm_1_joint/position -> command_interfaces:command_end_arm_1_joint/position
state_interfaces:state_start_arm_1_joint/position -> arm_controller:state_end_arm_1_joint/position, joint_state_broadcaster:state_end_arm_1_joint/position







