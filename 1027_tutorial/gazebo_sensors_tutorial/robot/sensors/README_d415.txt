Changes in the file _d415.urdf.xacro w.r.t. the one in 

1) Made as a xacro:macro in roder to set or not the color, ir and depth cameras
2) Corrected the hackBaseline arguments of the plugins:

    Color camera:
    <!--<hackBaseline>0.07</hackBaseline>-->
    <hackBaseline>${d415_cam_depth_to_color_offset}</hackBaseline>
                 
    Left ir camera:
    <!--<hackBaseline>0.07</hackBaseline>-->
    <hackBaseline>${d415_cam_depth_to_left_ir_offset}</hackBaseline>
                   
    Right ir_camera:
    <!--<hackBaseline>0.07</hackBaseline>-->
    <hackBaseline>${d415_cam_depth_to_right_ir_offset}</hackBaseline>
                    
    Depth camera:
    <!--<hackBaseline>0.07</hackBaseline>-->
    <hackBaseline>0.0</hackBaseline>
