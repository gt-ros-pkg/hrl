function shaving_init(){
    console.log("Begin Shaving Init");
    node.subscribe('/face_adls/global_move_poses', function(msg){
                    list_key_opts('#shave_list', msg)});
	node.subscribe('/ros_switch_state', function(msg){
                    window.tool_state=msg.data;});
	node.subscribe('/sm_selected_pose', function(msg){
                    window.sm_selected_pose = msg.data;});
    node.subscribe('/face_adls/controller_enabled', function(msg){
                    ell_controller_state_cb(msg)});
    var pubs = new Array();
    pubs['face_adls/global_move'] = 'std_msgs/String';
    pubs['ros_switch'] = 'std_msgs/Bool';
    pubs['pr2_ar_servo/find_tag'] = 'std_msgs/Bool';
    pubs['face_adls/local_move'] = 'std_msgs/String';
    pubs['face_adls/stop_move'] = 'std_msgs/Bool';
    pubs['pr2_ar_servo/tag_confirm'] = 'std_msgs/Bool';
    pubs['pr2_ar_servo/preempt'] = 'std_msgs/Bool';
    pubs['netft_gravity_zeroing/rezero_wrench'] = 'std_msgs/Bool';
    for (var i in pubs){
        advertise(i, pubs[i]);
    };
    node.subscribe('/pr2_ar_servo/state_feedback', function(msg){
                        servo_feedback_cb(msg);
                    });

    $('#servo_approach, #servo_stop').fadeTo(0,0.5);
    //$("#ft_view_widget").ft_viewer();
    $(".ell_control").hide();
    console.log("End Shaving Init");
};

function adj_mirror_cb(){
   log("calling mirror_adjust_service");
   node.rosjs.callService('/point_mirror',json('[]'),function(rsp){}); 
};

function servo_feedback_cb(msg){
    text = "Unknown result from servoing feedback";
    switch(msg.data){
        case 1:
            text = "Searching for AR Tag.";
            break
        case 2: 
            text = "AR Tag Found. CONFIRM LOCATION AND BEGIN APPROACH.";
            $('#servo_approach, #servo_stop').show().fadeTo(0,1);
            $('#servo_detect_tag').fadeTo(0,0.5);
            set_camera('ar_servo/confirmation_rotated');
            break
        case 3:
            text = "Unable to Locate AR Tag. ADJUST VIEW AND RETRY.";
            set_camera('ar_servo/confirmation_rotated');
            break
        case 4:
            text = "Servoing";
            break
        case 5:
            text = "Servoing Completed Successfully.";
            $('#servo_approach, #servo_stop').fadeTo(0,0.5);
            $('#servo_detect_tag').fadeTo(0,1);
            set_camera('kinect_head/rgb/image_color');
            break
        case 6:
            text = "Detected Collision with Arms while Servoing.  "+ 
                    "ADJUST AND RE-DETECT TAG.";
            $('#servo_approach, #servo_stop').fadeTo(0,0.5);
            $('#servo_detect_tag').fadeTo(0,1);
            break
        case 7:
            text = "Detected Collision in Base Laser while Servoing.  "+ 
                    "ADJUST AND RE-DETECT TAG.";
            $('#servo_approach, #servo_stop').fadeTo(0,0.5);
            $('#servo_detect_tag').fadeTo(0,1);
            break
        case 8:
            text = "View of AR Tag Was Lost.  ADJUST (IF NECESSARY) AND RE-DETECT.";
            $('#servo_approach, #servo_stop').fadeTo(0,0.5);
            $('#servo_detect_tag').fadeTo(0,1);
            set_camera('ar_servo/confirmation_rotated');
            break
        case 9:
            text = "Servoing Stopped by User. RE-DETECT TAG";
            $('#servo_approach, #servo_stop').fadeTo(0,0.5);
            $('#servo_detect_tag').fadeTo(0,1);
            set_camera('ar_servo/confirmation_rotated');
            break
    };
    log(text);
};

function ell_controller_state_cb(msg){
    if (msg.data){
        console.log("Ellipsoid Controller Active")
        $("#ell_controller").attr("checked","checked");
        $("#ell_cont_state_check").attr("aria-pressed",true).addClass('ui-state-active');
        $(".ell_control").show();
       } else {
        $("#ell_controller").attr("checked", "");
        console.log("Ellipsoid Controller Inactive")
        $("#ell_cont_state_check").attr("aria-pressed",false).removeClass('ui-state-active');
        $(".ell_control").hide();
       };
    };

function toggle_ell_controller(state){
    if (typeof state == 'undefined'){  
        if ($("#ell_controller").attr('checked')) {
            state = true;
            console.log("Ell Cont button active, sending true")
        } else {
            state = false;
            console.log("Ell Cont button inactive, sending false")
        };
    };
    console.log("Sending controller :"+state.toString());
    node.rosjs.callService('/face_adls/enable_controller',
                    '{"end_link":"%s_gripper_shaver45_frame","ctrl_params":"$(find hrl_face_adls)/params/l_jt_task_shaver45.yaml","enable":'+state+'}',
                    function(ret){
                        console.log("Switching Ell. Controller Returned");
                        ell_controller_state_cb(ret)}
                );
    };

function servo_detect_tag_cb(){
    node.publish('pr2_ar_servo/find_tag', 'std_msgs/Bool', json({'data':true}));
    console.log('Sending command to search for ARTag'); 
};

function servo_approach_cb(){
    node.publish('pr2_ar_servo/tag_confirm', 'std_msgs/Bool', json({'data':true}));
    console.log('Approaching Tag'); 
};

function servo_preempt(){
    node.publish('pr2_ar_servo/preempt', 'std_msgs/Bool', json({'data':true}));
};

function head_reg_cb(){
        $('#img_act_select').val('seed_reg');
        set_camera('head_registration/confirmation');
        alert('Click on your cheek in the video to register the ellipse.');
    };

function shave_stop(){
    node.publish('/face_adls/stop_move','std_msgs/Bool',json({'data':true}));
    };

function shave_step(cmd_str) {
    node.publish('/face_adls/local_move', 'std_msgs/String',
                json({'data':cmd_str}));
};

function send_shave_location(key) {
    node.publish('/face_adls/global_move', 'std_msgs/String', json({'data':key}));
};

function toggle_tool_power() {
    state = !window.tool_state;
    node.publish('ros_switch', 'std_msgs/Bool', json({'data':state}));
    console.log("Sending Tool Power Toggle");
};

function rezero_wrench(){
    node.publish('netft_gravity_zeroing/rezero_wrench','std_msgs/Bool',json({'data':true}));
    log("Sending command to Re-zero Force/Torque Sensor");
};
