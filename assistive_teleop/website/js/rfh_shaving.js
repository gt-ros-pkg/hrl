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
    node.subscribe('/shaving_state', function(msg){
                    shaving_feedback_cb(msg);}
                    );
    
    node.publish('wt_shave_step', 'geometry_msgs/Point', json({}));
    node.publish('wt_shave_location', 'std_msgs/Int8', json({}));
    node.publish('reg_confirm', 'std_msgs/Bool', json({}));
    node.publish('shaving_reg_confirm', 'std_msgs/Bool', json({}))
    node.publish('ros_switch', 'std_msgs/Bool', json({}));
    node.publish('pr2_ar_servo/arms_setup', 'std_msgs/Bool', json({}));
    node.publish('pr2_ar_servo/find_tag', 'std_msgs/Bool', json({}));
    node.publish('pr2_ar_servo/tag_confirm', 'std_msgs/Bool', json({}));
    node.publish('pr2_ar_servo/preempt', 'std_msgs/Bool', json({}));
    node.publish('netft_gravity_zeroing/rezero_wrench', 'std_msgs/Bool', json({}));
};

$(function(){
    $('.shave_acts, #shave_select, #shave_end').hide()
});

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
    node.rosjs.callService('/take_pc_snapshot',json(''),nop);
    $('#reg_confirm').show();
    log('USE R-VIZ TO FULLY ALIGN ELLIPSE, THEN CLICK CONFIRM.');
};

function reg_confirm_cb() {
    log("Sending command to untuck arms and start shaving.");
    $('#pc_snapshot').hide();
    $('#reg_confirm').hide();
    node.rosjs.callService('/setup_arms_shaving', json(''), function(msg){
        $('#tp').hide();
        $('#bpd2, #shave_select, #tool_power, #bpd2, #shave, #rezero_wrench').show();
        log('Untucking arms to begin shaving.');
        node.publish('shaving_reg_confirm', 'std_msgs/Bool', json({'data':true}));
        log("Sending command to prepare for shaving.");
        get_shave_side();
    });
};

function get_shave_side() {
    node.rosjs.callService('/rosbridge/get_param',
                           '["shaving_side"]',
                           function(msg){window.shaving_side = msg;});
};

function shave_step(x,y,z) {points = window.gm_point;
                        points.x = x; points.y = y; points.z = z;
                        node.publish('/wt_shave_step', 'geometry_msgs/Point', json(points));
                        log("Sending Adjustment to Shaving Position");
};

function send_shave_location(num) {
    log("Sending New Shave Position: "+ document.getElementById('shave_list').options[window.sm_selected_pose].text);
    if (window.shaving_side == 'r') {
        if (num >= 1 && num <= 6) {
            num += 7;
            };
    };
    node.publish('wt_shave_location', 'std_msgs/Int8', json({'data':num}));
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

function shaving_feedback_cb(msg){
    switch(msg.data){
        case 1:
            text = "Global Move Command Received";
            break
        case 2: 
            text = "Starting Small Move Up";
            break
        case 3: 
            text = "Starting Small Move Down";
            break
        case 4: 
            text = "Starting Small Move Left";
            break
        case 5: 
            text = "Starting Small Move Right";
            break
        case 6: 
            text = "Starting Small Move Out";
            break
        case 7: 
            text = "Starting Small Move In";
            break
        case 8: 
            text = "Moving in to Shave Current Position";
            break
        case 9: 
            text = "Collision While Moving";
            break
        case 10: 
            text = "Backing Away...";
            break
        case 11: 
            text = "Moving to new location...";
            break
        case 12: 
            text = "Approaching...";
            break
        case 13: 
            text = "Retreating Slowly";
            break
        case 14: 
            text = "Fast Retreat (Safety Threshold Exceeded)";
            break
        case 15: 
            text = "Beginning Force-Sensitive Hold.";
            break
    }
    log(text);
};

function shave_end_cb() {
$('#bpd2, #shave_select, #pc_snapshot, #tool_power, .ar_servo_controls, #ar_servoing_setup, #ar_servoing_done, #shave, #rezero_wrench').hide();
$('#tp, #shave_start').show();
$('#shave_end').hide();
log('Closed Shaving Interface');
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
