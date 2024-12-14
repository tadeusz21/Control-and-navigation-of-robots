Zadanie 3
węzeł gazebo_ros_state:
  Publishers:
    /link_states
    /model_states
  Service Servers:
    /get_entity_state:
    /set_entity_state:

Powyższe tematy oraz serwisy pozwalają na otrzymanie informacji o aktualnym stanie wszystkich modeli i linków w symulowanym świecie oraz ich zmiany.


Zadanie 5

[INFO] [1730749825.454011656] [hello_moveit]: układ bazowy (B): base_footprint
[INFO] [1730749825.454112182] [hello_moveit]: układ końcówki (E): arm_tool_link

B - base_footprint

E - arm_tool_link

F -  arm_7_link

orientation: 0.5; 0.5; 0.5; -0.5
position: 0; 0; 0.046

zad 6

ros2 service call /get_entity_state gazebo_msgs/srv/GetEntityState "{'name': 'green_cube_3', 'reference_frame': 'base_footprint'}"
waiting for service to become available...
requester: making request: gazebo_msgs.srv.GetEntityState_Request(name='green_cube_3', reference_frame='base_footprint')

response:
gazebo_msgs.srv.GetEntityState_Response(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=109, nanosec=6000000), frame_id='base_footprint'), state=gazebo_msgs.msg.EntityState(name='', pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.6129532489572659, y=0.12172421971793858, z=0.33512117754742166), orientation=geometry_msgs.msg.Quaternion(x=-2.121740371074175e-05, y=-0.0003902260631371974, z=0.014435026569939098, w=0.999895733204901)), twist=geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=-0.00034289905235218376, y=1.223409468826751e-05, z=0.011728505950795238), angular=geometry_msgs.msg.Vector3(x=0.0008709163850971656, y=0.0004891376063838345, z=0.00015024248149079038)), reference_frame=''), success=True)


ros2 service call /get_entity_state gazebo_msgs/srv/GetEntityState "{'name': 'tiago::arm_7_link', 'reference_frame': 'base_footprint'}"
requester: making request: gazebo_msgs.srv.GetEntityState_Request(name='tiago::arm_7_link', reference_frame='base_footprint')

response:
gazebo_msgs.srv.GetEntityState_Response(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=149, nanosec=599000000), frame_id='base_footprint'), state=gazebo_msgs.msg.EntityState(name='', pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.5810065155903941, y=0.12452790088114665, z=0.6040067554081324), orientation=geometry_msgs.msg.Quaternion(x=-0.015403366175301204, y=-0.9974549831560183, z=-0.0160488771360209, w=-0.06774013898989165)), twist=geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=-0.0018314937831165395, y=0.0010356306550164409, z=-0.011631544422857557), angular=geometry_msgs.msg.Vector3(x=-0.006333858546874671, y=0.002773370062450594, z=-0.0008824341661354643)), reference_frame=''), success=True)


ros2 service call /get_entity_state gazebo_msgs/srv/GetEntityState "{'name': 'tiago::arm_tool_link', 'reference_frame': 'base_footprint'}"
waiting for service to become available...
requester: making request: gazebo_msgs.srv.GetEntityState_Request(name='tiago::arm_tool_link', reference_frame='base_footprint')

response:
gazebo_msgs.srv.GetEntityState_Response(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), state=gazebo_msgs.msg.EntityState(name='', pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.0, y=0.0, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)), twist=geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0)), reference_frame=''), success=False)





ros2 service call /get_entity_state gazebo_msgs/srv/GetEntityState "{'name': 'green_cube_3', 'reference_frame': 'tiago::arm_link_7'}"
waiting for service to become available...
requester: making request: gazebo_msgs.srv.GetEntityState_Request(name='green_cube_3', reference_frame='tiago::arm_link_7')

response:
gazebo_msgs.srv.GetEntityState_Response(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), state=gazebo_msgs.msg.EntityState(name='', pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.0, y=0.0, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)), twist=geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0)), reference_frame=''), success=False)


zad 7
[INFO] [1730840366.728812133] [one_grasp]: Green Cube Position: x = 0.62, y = 0.10, z = 0.33

zad 8



