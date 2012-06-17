var head_pub;
var base_pub;
var scales={head:50,base:50,rarm:50,larm:50,rwrist:50,lwrist:50};
var arm_joints={right:[],left:[]}
var pointing_frame='openni_rgb_optical_frame'
var head_state;
var torso_state;
var plane = 'xy';

function teleop_init(){
	node.subscribe('/head_traj_controller_state_throttle',
                    function(msg){
                        window.head_state = msg.actual;
                        window.head_joints = msg.joint_names;
			        });
    node.subscribe('/r_arm_controller_state_throttle', function(msg){
                                       window.arm_joints.right = msg.actual});
    node.subscribe('/l_arm_controller_state_throttle', function(msg){
                                       window.arm_joints.left = msg.actual});

    //Arm Publishers
    var pubs = new Array()
    var sides = ["right","left"];
    for (var i=0; i < sides.length; i++){
        pubs['wt_'+sides[i]+'_arm_pose_commands'] = 'geometry_msgs/Point';
        pubs['wt_'+sides[i]+'_arm_angle_commands'] = 'trajectory_msgs/JointTrajectoryPoint';
        pubs['wt_lin_move_'+sides[i]] = 'std_msgs/Float32';
        pubs['wt_adjust_elbow_'+sides[i]] = 'std_msgs/Float32';
    };
        pubs['head_nav_goal']='geometry_msgs/PoseStamped'
        pubs['head_traj_controller/point_head_action/goal'] = 'pr2_controllers_msgs/PointHeadActionGoal'
        pubs['base_controller/command'] = 'geometry_msgs/Twist'
    for (var i in pubs){
        advertise(i, pubs[i]);
    };
};
$(function(){
	$('#scale_slider').slider({value:50,min:0,max:100,step:1,orientation:'vertical'}); 
	$("body").find('*').mouseup(function(e){window.base_pub = window.clearInterval(window.base_pub)});
});

function pub_lin_move(){
    node.publish('wt_lin_move_'+window.arm(),'std_msgs/Float32', json({"data":window.lin_move}));
    log('Sending '+window.arm().toUpperCase()+' Hand Advance/Retreat command');
};

function control_arm(x,y,z){
	goal = window.gm_point;
    goal.x = x;
    goal.y = y;
    goal.z = z;
    log('Sending command to '+window.arm().toUpperCase()+' arm');
    node.publish('wt_'+window.arm()+'_arm_pose_commands',
                 'geometry_msgs/Point', json(goal));
};

function teleop_arm() {
	var x,y,z,b9txt,b7txt;
	if (plane == 'xy'){
        x=0;
        y=1;
        z=2;
        b9txt='Up';
        b7txt='Down'
    } else if (plane == 'yz') {
        x=2;
        y=1;
        z=0;
        b9txt='Further Away';
        b7txt='Closer'
    };

    log('Controlling '+window.arm().toUpperCase()+' Arm');
    $('#scale_slider').unbind("slidestop").bind("slidestop", function(event,ui){scales[window.arm()[0]+'arm'] = $('#scale_slider').slider("value")});
    $('#scale_slider').show().slider("option", "value", scales[window.arm()[0]+'arm']);

    $("#tp").unbind().show();
    $("#tp").click(function(e){
        y = -(e.pageX - Math.round(this.width/2) - this.x)/500;
        x = -(e.pageY - Math.round(this.height/2) - this.y)/500;
        control_arm(x,y,0);	
    });

    $('#bpd_default').unbind();
    $('#bpd_default #b9').show().text(b9txt).click(function(e){
        control_arm(0,0,scales[window.arm()[0]+'arm']/500);
    });
    $('#bpd_default #b8').show().text("^").click(function(e){
        control_arm(scales[window.arm()[0]+'arm']/500,0,0);
    });
    $('#bpd_default #b7').show().text(b7txt).click(function(e){
        control_arm(0,0,-scales[window.arm()[0]+'arm']/500);
    });
    $('#bpd_default #b6').show().text(">").click(function(e){
        control_arm(0,-scales[window.arm()[0]+'arm']/500,0);
    });
    $('#bpd_default #b5').hide()
    $('#bpd_default #b4').show().text("<").click(function(e){
        control_arm(0,scales[window.arm()[0]+'arm']/500,0);
    });
    $('#bpd_default #b3').show().text("Advance").click(function(e){
        window.lin_move=0.1*(scales[window.arm()[0]+'arm']/100);
        pub_lin_move();
    });
    $('#bpd_default #b2').show().text("v").click(function(e){
        control_arm(-scales[window.arm()[0]+'arm']/500,0,0);
    });
    $('#bpd_default #b1').show().text("Retreat").click(function(e){
        window.lin_move=-0.1*(scales[window.arm()[0]+'arm']/100);
        pub_lin_move();
    });
};

function pub_elbow(dir) {
    dir =  (window.arm()=='right') ? -dir : dir; //Catch reflection, switch value for left side
    var action = (dir == 1) ? 'Raise ' : 'Lower ';
    node.publish('wt_adjust_elbow_'+window.arm(),'std_msgs/Float32', json({"data":dir}));
    log('Sending command to ' + action + window.arm().toUpperCase() + ' elbow')
};

function pub_arm_joints(angles){
    node.publish('wt_'+window.arm()+'_arm_angle_commands', 'trajectory_msgs/JointTrajectoryPoint', json(angles))
};

function teleop_wrist() {
    log('Controlling '+window.arm().toUpperCase()+' Hand');
    $('#scale_slider').unbind("slidestop").bind("slidestop", function(event,ui){scales[window.arm()[0]+'wrist'] = $('#scale_slider').slider("value")});
    $('#scale_slider').show().slider("option", "value", scales[window.arm()[0]+'wrist']);

    $("#tp").unbind().show();
    $("#tp").click(function(e){
        x = (e.pageX - Math.round(this.width/2) - this.x);
        y = (e.pageY - Math.round(this.height/2) - this.y);
        var joint_goals = arm_joints[window.arm()];
        joint_goals.positions[4] += x*0.0107;
        joint_goals.positions[5] -= y*(Math.PI/200);
        pub_arm_joints(joint_goals)
    });

    $('#bpd_default').unbind();
    $('#bpd_default #b9').show().text("Hand Roll Right").click(function(e){
        joint_goals = arm_joints[window.arm()];
        joint_goals.positions[6] += scales[window.arm()[0]+'wrist']*(Math.PI/200);
        pub_arm_joints(joint_goals)
    });
    $('#bpd_default #b8').show().text("Wrist Flex Out").click(function(e){
        joint_goals = arm_joints[window.arm()];
        joint_goals.positions[5] += scales[window.arm()[0]+'wrist']*0.0107;
        pub_arm_joints(joint_goals)
    });
    $('#bpd_default #b7').show().text("Hand Roll Left").click(function(e){
        joint_goals = arm_joints[window.arm()];
        joint_goals.positions[6] -= scales[window.arm()[0]+'wrist']*(Math.PI/200);
        pub_arm_joints(joint_goals)
    });
    $('#bpd_default #b6').show().text("Arm Roll Right").click(function(e){
        joint_goals = arm_joints[window.arm()];
        joint_goals.positions[4] += scales[window.arm()[0]+'wrist']*(Math.PI/200);
        pub_arm_joints(joint_goals)
    });
    $('#bpd_default #b5').hide()
    $('#bpd_default #b4').show().text("Arm Roll Left").click(function(e){
        joint_goals = arm_joints[window.arm()];
        joint_goals.positions[4] -= scales[window.arm()[0]+'wrist']*0.0107;
        pub_arm_joints(joint_goals)
    });
    $('#bpd_default #b3').show().text("Raise Elbow").click(function(e){
        pub_elbow(0.01*scales[window.arm()[0]+'wrist'])
    });
    $('#bpd_default #b2').show().text("Wrist Flex In").click(function(e){ 
        joint_goals = arm_joints[window.arm()];
        joint_goals.positions[5] -= scales[window.arm()[0]+'wrist']*(Math.PI/200);
        pub_arm_joints(joint_goals)
    });
    $('#bpd_default #b1').show().text("Lower Elbow").click(function(e){
        pub_elbow(-0.01*scales[window.arm()[0]+'wrist'])
    });
};


function pub_head_traj(head_traj_goal, dist){ //Send pan/tilt trajectory commands to head
		if (head_traj_goal.goal.trajectory.points[0].positions[0] < -2.70) {head_traj_goal.goal.trajectory.points[0].positions[0] = -2.70} 
		if (head_traj_goal.goal.trajectory.points[0].positions[0] > 2.70) {head_traj_goal.goal.trajectory.points[0].positions[0] = 2.70}
		if (head_traj_goal.goal.trajectory.points[0].positions[1] < -0.5) {head_traj_goal.goal.trajectory.points[0].positions[1] = -0.5}
		if (head_traj_goal.goal.trajectory.points[0].positions[1] > 1.4) {head_traj_goal.goal.trajectory.points[0].positions[1] = 1.4}
		head_traj_goal.goal.trajectory.joint_names = window.head_joints;
		head_traj_goal.goal.trajectory.points[0].velocities = [0, 0];
		head_traj_goal.goal.trajectory.points[0].time_from_start.secs = Math.max(4*dist, 1);
  		node.publish('head_traj_controller/joint_trajectory_action/goal', 'pr2_controllers_msgs/JointTrajectoryActionGoal', json(head_traj_goal));
};

function pub_head_goal(x,y,z,frame) { //Send 3d point to look at using kinect
	node.publish('head_traj_controller/point_head_action/goal', 'pr2_controllers_msgs/PointHeadActionGoal', json(
		{  'goal':{'target':{'header':{'frame_id':frame },
				             'point':{'x':x, 'y':y, 'z':z}},
                   'pointing_axis':{'x':0, 'y':0, 'z':1},
                   'pointing_frame':window.pointing_frame,
                   'max_velocity':0.35}}))
	//log("Sending command to looking at "+x.toString() +", "+y.toString()+", "+z.toString()+" in "+frame);
};

function teleop_head() {
	window.head_pub = window.clearInterval(head_pub);
	log('Controlling Head');
	$('#scale_slider').unbind("slidestop").bind("slidestop", function(event,ui){scales.head = $('#scale_slider').slider("value")});
	$('#scale_slider').show().slider("option", "value", scales.head);
	
	$("#tp").unbind().show();
	$("#tp").click(function(e){
		window.head_pub = window.clearInterval(head_pub);
		x = (e.pageX - Math.round(this.width/2) - this.x)/200;
		y = (e.pageY - Math.round(this.height/2) - this.y)/200;
   		head_traj_goal = JTAGoal;
		head_traj_goal.goal.trajectory.points[0] = window.head_state;
		head_traj_goal.goal.trajectory.points[0].positions[0] -= x;
		head_traj_goal.goal.trajectory.points[0].positions[1] += y;
                pub_head_traj(head_traj_goal, Math.sqrt(x*x+y*y));
	});
	
	$('#bpd_default').unbind();
	$('#b9, #b7', '#bpd_default').hide(); 
	$('#bpd_default #b8').show().text("^").click(function(e){//head up 
		window.head_pub = window.clearInterval(head_pub);
   		head_traj_goal = JTAGoal;
		head_traj_goal.goal.trajectory.points[0] = window.head_state;
		head_traj_goal.goal.trajectory.points[0].positions[1] -= scales.head/150;
                pub_head_traj(head_traj_goal, scales.head/150);
	});
	$('#bpd_default #b6').show().text(">").click(function(e){ //head right
		window.head_pub = window.clearInterval(head_pub);
   		head_traj_goal = JTAGoal;
		head_traj_goal.goal.trajectory.points[0] = window.head_state;
		head_traj_goal.goal.trajectory.points[0].positions[0] -= scales.head/150;
                pub_head_traj(head_traj_goal, scales.head/150);
	});
	$('#bpd_default #b5').show().text("_|_").click(function(e){ //center head to (0,0)
		window.head_pub = window.clearInterval(head_pub);
        pub_head_goal(0.8, 0.0, -0.25, '/base_footprint');
	});
	$('#bpd_default #b4').show().text("<").click(function(e){ //head left
		window.head_pub = window.clearInterval(head_pub);
   		head_traj_goal = JTAGoal;
		head_traj_goal.goal.trajectory.points[0] = window.head_state;
		head_traj_goal.goal.trajectory.points[0].positions[0] += scales.head/150;
                pub_head_traj(head_traj_goal, scales.head/150);
	});
	$('#bpd_default #b3').show().text("Track Right Hand").click(function(e){
		window.head_pub = window.clearInterval(head_pub);
		window.head_pub = window.setInterval("pub_head_goal(0,0,0,'r_gripper_tool_frame');",200);
	});	
  	$('#bpd_default #b2').show().text("v").click(function(e){ //head down
		window.head_pub = window.clearInterval(head_pub);
   		head_traj_goal = JTAGoal;
		head_traj_goal.goal.trajectory.points[0] = window.head_state;
		head_traj_goal.goal.trajectory.points[0].positions[1] += scales.head/150;
                pub_head_traj(head_traj_goal, scales.head/150);
	});
	$('#bpd_default #b1').show().text("Track Left Hand").click(function(e){
		window.head_pub = window.clearInterval(head_pub);
		window.head_pub = window.setInterval("pub_head_goal(0,0,0,'l_gripper_tool_frame');",200);
	});
};

function pub_Twist(bx,by,bz) {
        var date = new Date();
        //log('Publishing base cmd at '+date.getMilliseconds().toString());
        node.publish('base_controller/command', 'geometry_msgs/Twist',
                    '{"linear":{"x":'+bx+',"y":'+by+',"z":0},"angular":{"x":0,"y":0,"z":'+bz+'}}');
};


function start_base_pub(bx,by,bz) {
    bx = 0.002*scales.base*bx
    by = 0.002*scales.base*by
    bz = 0.006*scales.base*bz
	window.base_pub = setInterval("pub_Twist("+bx+","+by+","+bz+")", 100);
};

function teleop_base() {
	log("Controlling Base");
	$('#scale_slider').unbind("slidestop").bind("slidestop", function(event,ui){scales.base = $('#scale_slider').slider("value")});
	$('#scale_slider').show().slider("option", "value", scales.base);
	$("#tp").unbind().hide();
	$('#bpd_default').unbind();
	$('#b9, #b7, #b5','#bpd_default ').hide()
    $('#b8, #b6, #b4, #b3, #b2, #b1', '#bpd_default').mouseout(function(e){
                               window.base_pub = window.clearInterval(base_pub);
                                    }).mouseup(function(e){
                               window.base_pub = window.clearInterval(base_pub);
                                    });
  	$('#bpd_default #b8').show().text("^").mousedown(function(e){
                                 start_base_pub(1,0,0);
    });
	$('#bpd_default #b6').show().text(">").mousedown(function(e){
                                 start_base_pub(0,-1,0);
    });
	$('#bpd_default #b4').show().text("<").mousedown(function(e){
                                start_base_pub(0,1,0);
    });
  	$('#bpd_default #b3').show().text("Turn Right").mousedown(function(e){
                               start_base_pub(0,0,-1);
    });
	$('#bpd_default #b2').show().text("v").mousedown(function(e){
                                start_base_pub(-1,0,0)
    });
   	$('#bpd_default #b1').show().text("Turn Left").mousedown(function(e){
                                start_base_pub(0,0,1)
    });
};
