#!/usr/local/bin/python

import sys
import subprocess
import os
import time
import glob
import roslib
roslib.load_manifest('hrl_tactile_controller')
import rospy
import hrl_lib.util as ut
import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs

mpc_state = ''
mpc_error = ''

def check_files(folder):
    if not os.path.isfile(folder+'/overall_result.pkl'):
        print "no overall results file"
        return False
    elif glob.glob(folder+'/*reach_from*to_goal*log*.pkl') == []:
        print "no controller logging files"
        return False
    else:
        file_list = glob.glob(folder+'/*pkl')
        for pkl_file in file_list:
            try:
                data = ut.load_pickle(pkl_file)
            except:
                print "failed to load pickle, corrupted? ..."
                return False
            if data == None:
                print "failed to load pickle, none there? ..."
                return False
        print "file exists"
        return True

def check_running(folder):
    if not os.path.isfile(folder+'/running.txt'):
        print "no other machine is running this trial"
        return False
    else:
        print "another machine IS running this trial"
        return True

# Store the state information from the monitor node. Allows the control to be somewhat stateful.
def mpcMonitorCallback(msg):
    global mpc_state, mpc_error
    mpc_state = msg.state
    mpc_error = msg.error


if __name__ == '__main__':  
    rospy.init_node('single_run_script')
    rospy.Subscriber("/haptic_mpc/mpc_state", haptic_msgs.HapticMpcState, mpcMonitorCallback)

    if sys.argv[1] == "":
        print "Give name of reach_problem_dict"
        sys.exit()

    ###############################################################################################################################################
    ########## all of these options should eventually be available in simulation batch mode, but for now this is OK - marc ########################
    ###############################################################################################################################################
    if sys.argv[2] == "use_skin":
        CONTROLLER_SWITCH=''
        SENSING="taxels"        
    elif sys.argv[2] == "use_ft_sensor":
        CONTROLLER_SWITCH=''
        SENSING="ft_at_base"
    elif sys.argv[2] == "ignore_skin":
        CONTROLLER_SWITCH='--is'
        SENSING="taxels"
    else:
        print 'Specify either use_skin or use_ft_sensor or ignore_skin'
        sys.exit()

    if sys.argv[3] == "single_reach":
        REACH_SWITCH='--single_reach'
    elif sys.argv[3] == "multiple_reaches":
        REACH_SWITCH=''
    else:
        print 'Specify either single_reach or multiple_reaches'
        sys.exit()

    if sys.argv[4] == "":
        print 'Specify an allowable force'
        sys.exit()

    if sys.argv[5] == "":
        print "need to specify which simulation arm to use"
        sys.exit()
    elif sys.argv[5] == "sim3_with_hand":
        d_robot = '--three_link_with_hand'
    elif sys.argv[5] == "sim3":
        d_robot = '--planar_three_link_capsule'
    ###############################################################################################################################################
    ########## all of these options should eventually be available in simulation batch mode, but for now this is OK - marc ########################
    ###############################################################################################################################################


    intermed_name = (os.path.split(sys.argv[1])[1]).split('_')[3:6]

    dir_name = intermed_name[0]+'_'+intermed_name[1]+'_'+intermed_name[-1].split('.')[0]

    os.system('mkdir '+dir_name) 
    os.chdir(dir_name) 

    check = False
    count = 0

    if check_files('./'):
        print "#############################################################################"
        print "Result pkl exists. Ignoring this trial :" + dir_name  
        print "#############################################################################"
        sys.exit()

    if check_running('./'):
        print "#############################################################################"
        print "Running already on another machine, ignoring this trial :" + dir_name  
        print "#############################################################################"
        sys.exit()

    os.system('touch running.txt')

    not_done = True

    while not_done == True:
        count = count + 1

        try:
            rospy.delete_param('/launch')
        except KeyError:
            print "value not set"
        try:
            rospy.delete_param('/m3')
        except KeyError:
            print "value not set"

        rospy.set_param('use_sim_time', True)
        rospy.set_param('/m3/software_testbed/resolution', 100)
        rospy.set_param('use_prox_sensor', True )

        # Load obstacles and params for the ODE simulator.
        subprocess.Popen(['rosrun', 
                          'hrl_common_code_darpa_m3', 
                          'obstacles.py', 
                          '--pkl='+sys.argv[1]])

        subprocess.Popen(['rosrun', 
                          'hrl_software_simulation_darpa_m3', 
                          'sim_arm_param_upload.py', 
                          d_robot])

        time.sleep(3)

        simulator = subprocess.Popen(['rosrun',
                                      'hrl_software_simulation_darpa_m3',
                                      'simulator',
                                      '/skin/contacts:=/skin/contacts_unused',
                                      '__name:=software_simulation_node'])

        time.sleep(5)


        subprocess.Popen(['roslaunch', 'hrl_tactile_controller', 'params.launch'])

        time.sleep(2)     
        #sim_visualisation = subprocess.Popen(['roslaunch', 'hrl_software_simulation_darpa_m3', 'ode_sim_viz.launch'])

        haptic_state = subprocess.Popen(['rosrun', 
                                         'hrl_tactile_controller', 
                                         'robot_haptic_state_node.py',
                                         '-r','sim3',
                                         '-s','fabric', 
                                         '-a','r'])

        traj_manager = subprocess.Popen(['rosrun', 
                                         'hrl_tactile_controller',
                                         'arm_trajectory_generator.py',
                                         '-r', 'sim3',
                                         '-s', 'fabric',
                                         '-a', 'r'])

        time.sleep(2)     

        cvxgen_node = subprocess.Popen(['rosrun', 
                                         'sandbox_marc_darpa_m3', 
                                         'proximity_3_link_qs_mpc'])

        monitor = subprocess.Popen(['rosrun', 
                                    'hrl_tactile_controller', 
                                    'haptic_mpc_monitor.py'])

        logger = subprocess.Popen(['rosrun', 
                                   'hrl_tactile_controller', 
                                   'haptic_mpc_logger.py'])

        time.sleep(2)     

        controller = subprocess.Popen(['rosrun', 
                                      'sandbox_marc_darpa_m3',
                                      'test_proximity_sensor.py',
                                      '--sim'])

        keep_running = True
        outcome = None

        try:
            while keep_running:
                rospy.sleep(0.001)
                if ('reached_goal' in mpc_state):
                    keep_running = False
                    outcome = True
                elif ('unsuccessful' in mpc_state):
                    keep_running = False                
                    outcome = False
                elif ('attempt_timeout' in mpc_error):
                    keep_running = False                
                    outcome = False
                elif ('max_force_exceeded' in mpc_error):
                    keep_running = False                
                    outcome = False
                if keep_running == False:
                    print "mpc_state :\n", mpc_state
                    print "mpc_error :\n", mpc_error
                if controller.poll() != None or cvxgen_node.poll() != None or traj_manager.poll() != None or haptic_state.poll() != None or logger.poll() != None or monitor.poll() != None or simulator.poll() != None:
                    raise ValueError("ProcessDead")

            rospy.loginfo('Controller is done ...')

            controller.kill()
            cvxgen_node.kill()

            time.sleep(20)

            traj_manager.kill()
            haptic_state.kill()
            logger.kill()
            monitor.kill()
            simulator.kill()


            rospy.loginfo('killed all processes.')
            d = {}
            d['reached_goal'] = outcome
            d['state'] = mpc_state
            d['error'] = mpc_error
            ut.save_pickle(d, "overall_result.pkl")

        except:
            rospy.loginfo('Uh-oh, somethings gone wrong ...')

            try:
                controller.kill()
                traj_manager.kill()
                haptic_state.kill()
                logger.kill()
                monitor.kill()
                simulator.kill()
                cvxgen_node.kill()
            except:
                pass
            
            rospy.loginfo('killed all processes.')



        # not sure what to do here.
        # try:
        #   while True:
        #     time.sleep(5.0)
        # except KeyboardInterrupt:
        #   controller.terminate()
        #   haptic_state.terminate()
        #   logger.terminate()
        #   monitor.terminate()
        #   simulator.terminate()
        #   cvxgen_node.terminate()
        #   # sim_visualisation.terminate()
        #   # skin_msg.terminate()
        #   sys.exit()



        # subprocess.Popen(['rosnode', 'kill', 'software_simulation_node'])
        # subprocess.Popen(['rosnode', 'kill', 'taxel_array_to_skin_contact'])
        # subprocess.Popen(['rosnode', 'kill', 'skin_contact_to_resultant_force'])
        # subprocess.Popen(['rosnode', 'kill', 'ode_log_and_monitor_node'])
        # subprocess.Popen(['rosnode', 'kill', 'contact_memory_node'])

        if check_files('./'):
            not_done = False
            check = True
        elif count >= 3:
            not_done = False

    os.system('rm running.txt')

    if check == False:
        if os.path.isfile('../unrun_trials.pkl'):
            data = ut.load_pickle("../unrun_trials.pkl")
            data['unrun_trials'].append(dir_name)
            ut.save_pickle(data, "../unrun_trials.pkl")
        else:
            data = {'unrun_trials':[]}
            data['unrun_trials'].append(dir_name)
            ut.save_pickle(data, "../unrun_trials.pkl")
