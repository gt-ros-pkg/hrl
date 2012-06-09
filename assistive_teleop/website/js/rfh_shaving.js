function shaving_init(){
	node.subscribe('/ros_switch_state', function(msg){
                    window.tool_state=msg.data;
                  });
	node.subscribe('/sm_selected_pose', function(msg){
                    window.sm_selected_pose = msg.data}
                    );
    node.subscribe('/pr2_ar_servo/state_feedback', function(msg){
                    servo_feedback_cb(msg);}
                    );
   // node.subscribe('/face_adls/controller_state', function(msg){
   //                 shaving_feedback_cb(msg);}
   //                 );
    node.subscribe('/face_adls/controller_enabled', function(msg){
                    ell_controller_state_cb(msg);}
                    );
    node.subscribe('/face_adls/global_move_poses', function(msg){
                    list_key_opts('#shave_list', msg)}
                    );
    node.publish('/face_adls/local_move', 'std_msgs/String', json({}));
    node.publish('/face_adls/global_move', 'std_msgs/String', json({}));
    node.publish('reg_confirm', 'std_msgs/Bool', json({}));
    node.publish('shaving_reg_confirm', 'std_msgs/Bool', json({}))
    node.publish('ros_switch', 'std_msgs/Bool', json({}));
    node.publish('pr2_ar_servo/arms_setup', 'std_msgs/Bool', json({}));
    node.publish('pr2_ar_servo/find_tag', 'std_msgs/Bool', json({}));
    node.publish('pr2_ar_servo/tag_confirm', 'std_msgs/Bool', json({}));
    node.publish('pr2_ar_servo/preempt', 'std_msgs/Bool', json({}));
    node.publish('netft_gravity_zeroing/rezero_wrench', 'std_msgs/Bool', json({}));
    //set_ft_ref_bar();
};

function ell_controller_state_cb(msg){
    if (msg.data){
        log("Ellipsoid Controller Active")
        $("#ell_controller").attr("checked","checked");
        $("#ell_cont_state_check").attr("aria-pressed",true).addClass('ui-state-active');

       } else {
        $("#ell_controller").attr("checked", "");
        log("Ellipsoid Controller Inactive")
        $("#ell_cont_state_check").attr("aria-pressed",false).removeClass('ui-state-active');
       };
    };

function toggle_ell_controller(){
    if ($("#ell_controller").attr('checked')) {
        state = true;
    } else {
        state = false;
    };
    node.rosjs.callService('/face_adls/enable_controller',
                    '{"end_link":"%s_gripper_shaver45_frame","ctrl_params":"$(find hrl_face_adls)/params/l_jt_task_shaver45.yaml","enable":'+state+'}',
                    nop);
    };

function servo_setup_cb(){
    log('Sending command to prep arms for ar-servoing approach');
    node.rosjs.callService('/pr2_ar_servo/arms_setup','[""]',function(msg){
        $('.ar_servo_controls').show();
        $('#ar_servoing_setup, #servo_approach, #servo_stop').hide();
        set_camera('/l_forearm_cam/image_color_rotated')
        log('USE BUTTONS TO FIND TAG AND PERFORM APPROACH');
        })
};

function servoing_done_cb(){
    node.rosjs.callService('/setup_registration','[""]',function(msg){
        $('#pc_snapshot').show();
        $('#ar_servoing_done, .ar_servo_controls').hide(); 
        set_camera('/kinect_head/rgb/image_color');
        log('Approach Completed. ENSURE ROBOT IS LOOKING AT YOUR HEAD, PLACE YOUR HEAD IN NEUTRAL POSITION, FREEZE POINTCLOUD')
    })
};

function servo_detect_tag_cb(){
    node.publish('pr2_ar_servo/find_tag', 'std_msgs/Bool', json({'data':true}));
    log('Sending command to search for ARTag'); 
};

function servo_approach_cb(){
    node.publish('pr2_ar_servo/tag_confirm', 'std_msgs/Bool', json({'data':true}));
    log('Approaching Tag'); 
};

function servo_preempt(){
    node.publish('pr2_ar_servo/preempt', 'std_msgs/Bool', json({'data':true}));
};

function servo_feedback_cb(msg){
    switch(msg.data){
        case 1:
            text = "Searching for AR Tag."
            break
        case 2: 
            text = "AR Tag Found. BEGIN APPROACH.";
            $('#servo_approach, #servo_stop, #ar_servoing_done').show().fadeTo(0,1);
            $('#servo_detect_tag').fadeTo(0,0.5);
            set_camera('/kinect_head/rgb/image_color')
            break
        case 3:
            text = "Unable to Locate AR Tag. ADJUST VIEW AND RETRY.";
            set_camera('/l_forearm_cam/image_color_rotated')
            break
        case 4:
            text = "Servoing"
            break
        case 5:
            text = "Servoing Completed Successfully.";
            $('#servo_approach, #servo_stop').fadeTo(0,0.5);
            $('#servo_detect_tag').fadeTo(0,1);
            set_camera('/kinect_head/rgb/image_color')
            break
        case 6:
            text = "Servoing Detected Collision with Arms.  "+ 
                    "ADJUST AND RE-DETECT TAG.";
            $('#servo_approach, #servo_stop').fadeTo(0,0.5);
            $('#servo_detect_tag').fadeTo(0,1);
            break
        case 7:
            text = "Servoing Detected Collision in Base Laser.  "+ 
                    "ADJUST AND RE-DETECT TAG.";
            $('#servo_approach, #servo_stop').fadeTo(0,0.5);
            $('#servo_detect_tag').fadeTo(0,1);
            break
        case 8:
            text = "Lost view of AR Tag.  ADJUST AND RE-DETECT."
            $('#servo_approach, #servo_stop').fadeTo(0,0.5);
            $('#servo_detect_tag').fadeTo(0,1);
            set_camera('/l_forearm_cam/image_color_rotated')
            break
        case 9:
            text = "Servoing Stopped by User. RE-DETECT TAG";
            $('#servo_approach, #servo_stop').fadeTo(0,0.5);
            $('#servo_detect_tag').fadeTo(0,1);
            set_camera('/l_forearm_cam/image_color_rotated')
            break
    }
    log(text);
};
 
function pc_snapshot_cb() {
    empty_srv('/take_pc_snapshot')
    $('#reg_confirm').show();
    log('USE R-VIZ TO FULLY ALIGN ELLIPSE, THEN CLICK CONFIRM.');
};

function head_reg_cb(){
        window.img_act = 'seed_reg';
        alert('Click your head in the video to seed the ellipse registratoin')
    };

function reg_confirm_cb() {
    log("Sending command to untuck arms and start shaving.");
    $('#pc_snapshot').hide();
    $('#reg_confirm').hide();
    node.rosjs.callService('/setup_arms_shaving', json(''), function(msg){
        $('#tp').hide();
        $('#bpd_ell_trans, #bpd_ell_rot, #shave_select, #tool_power, #shave, #rezero_wrench').show();
        log('Untucking arms to begin shaving.');
        node.publish('shaving_reg_confirm', 'std_msgs/Bool', json({'data':true}));
        log("Sending command to prepare for shaving.");
        get_shave_side();
    });
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
    log("Sending Tool Power Toggle");
};

function rezero_wrench(){
    node.publish('netft_gravity_zeroing/rezero_wrench','std_msgs/Bool',json({'data':true}));
    log("Sending command to Re-zero Force/Torque Sensor");
};

function shave_start_cb(){
    $('#shave_end, #ar_servoing_setup').show();
    $('#shave_start').hide();
    log('Opened Shaving Interface');
};

//function shaving_feedback_cb(msg){
//    text = "Received controller state msg not fitting a known case"
//    switch(msg.data){
//        case 1:
//            text = "Global Move Command Received";
//            break
//        case 2: 
//            text = "Starting Small Move Up";
//            break
//        case 3: 
//            text = "Starting Small Move Down";
//            break
//        case 4: 
//            text = "Starting Small Move Left";
//            break
//        case 5: 
//            text = "Starting Small Move Right";
//            break
//        case 6: 
//            text = "Starting Small Move Out";
//            break
//        case 7: 
//            text = "Starting Small Move In";
//            break
//        case 8: 
//            text = "Moving in to Shave Current Position";
//            break
//        case 9: 
//            text = "Collision While Moving";
//            break
//        case 10: 
//            text = "Backing Away...";
//            break
//        case 11: 
//            text = "Moving to new location...";
//            break
//        case 12: 
//            text = "Approaching...";
//            break
//        case 13: 
//            text = "Retreating Slowly";
//            break
//        case 14: 
//            text = "Fast Retreat (Safety Threshold Exceeded)";
//            break
//        case 15: 
//            text = "Beginning Force-Sensitive Hold.";
//            break
//    }
//    log(text);
//};

function shave_end_cb() {
$('#bpd_ell_trans, #bpd_ell_rot, #shave_select, #pc_snapshot, #tool_power, .ar_servo_controls, #ar_servoing_setup, #ar_servoing_done, #shave, #rezero_wrench').hide();
$('#tp, #shave_start').show();
$('#shave_end').hide();
log('Closed Shaving Interface');
};

function set_ft_ref_bar(){
    node.rosjs.callService('/rosbridge/get_param','["face_adls_manager"]',
          function(params){
              var danger_pct = ((15-params['dangerous_force_thresh'])/15)*100;
              var act_pct = ((15-params['activity_force_thresh'])/15)*100-danger_pct;
              $("#ft_ref_danger").height(danger_pct.toString()+'%');
              $("#ft_ref_danger_label").text("Danger\r\n >"+params.dangerous_force_thresh.toString()+" N")
              $("#ft_ref_act").height(act_pct.toString()+'%');
              $("#ft_ref_act_label").text("Activity\r\n  >"+params.activity_force_thresh.toString()+ "N")
              $("#ft_ref_null").height('100%');
              });
    };

function set_ft_bar(pct,yellow_pct){
    g = r = "FF";
    if (pct > yellow_pct){
        g = Math.round((255*(1-(pct-yellow_pct)/(1-yellow_pct)))).toString(16);
        if (g.length==1){g="0"+g};
    };
    if (pct < yellow_pct){
        r = (Math.round(255*(pct/yellow_pct))).toString(16);
        if (r.length==1){ r="0"+r};
    };
    color = "#"+r+g+'00';
    $('#ft_bar_wrapper').css('background-color', color);
    $('#ft_bar_value').css('height', Math.round(100*(1-pct)).toString()+'%');
};
