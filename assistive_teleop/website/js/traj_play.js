var traj_actions = ['Shaving Left Cheek', 'Shaving Right Cheek', 'Servoing']
var traj_arms = ['Left','Right']

function traj_play_init(){
    console.log("Begin Traj Play Init");
    var traj_play_act_spec = new ros.actionlib.ActionSpec('pr2_traj_playback/TrajectoryPlayAction');
    window.traj_play_r_client = new ros.actionlib.SimpleActionClient(node,'/trajectory_playback_r', traj_play_act_spec);
    window.traj_play_l_client = new ros.actionlib.SimpleActionClient(node,'/trajectory_playback_l', traj_play_act_spec);

    traj_play_r_client.wait_for_server(10, function(e){
          if(!e) {
            log("Couldn't find right trajectory playback action server.");
          } else {
            console.log("Found Right Trajectory Playback Action");
          };
      });
    traj_play_l_client.wait_for_server(10, function(e){
          if(!e) {
            log("Couldn't find left trajectory playback action server.");
          } else {
            console.log("Found Left Trajectory Playback Action");
          };
      });
    console.log("End Traj Play Init");

    if (window.get_param_free){
        window.get_param_free = false;
        console.log("Traj play has locked get_param");
        node.rosjs.callService('/rosbridge/get_param','["face_adls_traj_files"]',
                      function(msg){window.face_adls_params = msg;
                                    list_opts('#traj_play_select', msg);
                                    window.get_param_free = true;
                                    console.log("Traj play has released get_param");
                                    });
    } else {
          console.log("Traj Play tab waiting for rosparam service");
          setTimeout('init_params()',500);
    };
    init_TrajPlayGoal();
};

$(function(){
    $("#traj_play_radio").buttonset();
    });

function init_TrajPlayGoal(){
	if (window.get_msgs_free){
        window.get_msgs_free = false;
        console.log('Locking for TrajPlayGoal');
		node.rosjs.callService('/rosbridge/msgClassFromTypeString',
                          json(["pr2_traj_playback/TrajectoryPlayGoal"]),
                          function(msg){window.TrajPlayGoal=msg;
                                        window.get_msgs_free = true;
                                        console.log('Unlocking: Got TrajPlayGoal');
                          });
	} else {
        console.log("TrajPlayGoal Waiting for msg lock");
        setTimeout(function(){init_TrajPlayGoal();},500);
    }
};
function traj_play_send_goal(){
    goal = window.TrajPlayGoal;
    goal.mode = parseInt($('input[name=traj_play_radio]:checked','#traj_play_radio').val());
    goal.reverse = $("#traj_play_reverse").is(":checked");
    goal.setup_velocity = 0.1;
    goal.traj_rate_mult = 0.8;
    goal.filepath = $("#traj_play_select option:selected").val()
    if ($("#traj_play_select option:selected").text().slice(1,2) == 'R'){
        window.traj_play_r_client.send_goal(goal)
    } else if ($("#traj_play_select option:selected").text().slice(1,2) == 'L'){
        window.traj_play_l_client.send_goal(goal)
    } else {log("Cannot Tell Which Arm to use based on filepath")};
    console.log("Sending Trajectory Play Goal");
};

