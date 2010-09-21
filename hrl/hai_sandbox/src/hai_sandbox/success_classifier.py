import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import glob
import hrl_lib.rutils as ru
import hrl_pr2_lib.devices as hpr2
import hai_sandbox.bag_processor as bp
import hrl_lib.util as ut
import visualization_msgs.msg as vm
import numpy as np
import hai_sandbox.viz as viz
import sensor_msgs.msg as sm

import pdb

def success_failure_classification_preprocess(folder_name):
    data_dict = None
    #counter = 0
    for bag_name in glob.glob('%s/*.bag' % folder_name):
        print 'Loading bag %s' % bag_name
        topics_dict = ru.bag_sel(bag_name, 
                                    ['/imitate_behavior_marker',
                                    '/pressure/l_gripper_motor',      
                                    '/pressure/r_gripper_motor',
                                    '/accelerometer/l_gripper_motor', 
                                    '/accelerometer/r_gripper_motor',
                                    '/joint_states', 
                                    '/l_cart/command_pose', 
                                    '/r_cart/command_pose'])

        ##break each mat into segments based on states
        #/imitate_behavior_marker
        #topics_dict['/imitate_behavior_marker']['t']
        print 'There are %d marker messages.' % len(topics_dict['/imitate_behavior_marker']['msg'])
        time_segments = [[]]
        for marker_msg in topics_dict['/imitate_behavior_marker']['msg']:
            if len(time_segments[-1]) >= 2:
                time_segments.append([])
            time_segments[-1].append(marker_msg.header.stamp.to_time())

        if data_dict == None:
            data_dict = {}
            #organize by segment, then by type, then by what that type will store
            for i in range(len(time_segments)):
                data_dict[i] = {}
        
        ###pressure mat with times
        ##/pressure/l_gripper_motor      
        ##/pressure/r_gripper_motor
        for ptop in ['/pressure/l_gripper_motor', '/pressure/r_gripper_motor']:
            p = topics_dict[ptop]
            psegs = bp.segment_msgs(time_segments, p['msg'])
            print '>> Separated records of %s (%s) into %d segments' % (ptop, bag_name, len(psegs))
            #convert segments into mats
            for i, pseg in enumerate(psegs):
                #each col is an example in left-f and right_f
                left_f, right_f, ptimes = hpr2.pressure_state_to_mat(pseg)
                if not data_dict[i].has_key(ptop):
                    data_dict[i][ptop] = []
                data_dict[i][ptop].append({'left': left_f, 'right': right_f, 't': ptimes})


        for ptop in ['/pressure/l_gripper_motor', '/pressure/r_gripper_motor']:
            for k in data_dict.keys():
                print '>>', k, ptop, len(data_dict[k][ptop])
        #for i in range(len(data_dict[i])
        #print data_dict[i]
            #topics = ['/pressure/l_gripper_motor', '/pressure/r_gripper_motor']
        #counter = counter + 1
        #print counter
        #if counter > 2:
        #    break
        ########################################
        ## ignore everything else for now
        ########################################
        ##accelerometer mat with times
        #/accelerometer/l_gripper_motor 
        #/accelerometer/r_gripper_motor

        #
        ##joint effort mat with times
        #/joint_states 
        #
        ##cart_mat with times
        #/l_cart/command_pose 
        #/r_cart/command_pose 
        #
        ##ignore this one for now
        #/l_cart/state 
        #/r_cart/state
    return data_dict

class PCAKNeighborClassifier:

    def __init__(self):
        pass

    def learn_model(self, X, Y):
        self.X = X
        self.Y = Y
        #svd X
        #construct kd tree

    def classify(self, x):
        #project using svd
        #look up in kd tree, return results
        pass


class TimeSeriesClassifier:

    def __init__(self):
        pass

    def learn_model(self, succ_pickle, fail_pickle):
        # load in pickle
        successes = ut.load_pickle(succ_pickle)
        failures = ut.load_pickle(fail_pickle)

        def segment_pressure_dataset(pdict):
            topics = ['/pressure/l_gripper_motor', '/pressure/r_gripper_motor']
            for topic in topics:
                #find longest record in terms of number of messages
                shortest_left = 0
                shortest_right = 0
                for record in pdict[topic]:
                    shortest_left = min(record['left'].shape[1], shortest_left)
                    shortest_right = min(record['right'].shape[1], longest_right)
                    #pdb.set_trace()
                    #record['t'][0]

                ##break up existing records
                #for record in pdict[topic]:
                #    record['left']

        # for each state
        for state_numb in range(len(successes.keys())):
            #data_dict[i][ptop].append({'left': left_f, 'right': right_f, 't': ptimes})
            segment_pressure_dataset(successes[state_numb])
            failures[state_numb]
            pdb.set_trace()

        #       for each record of success and failure
        #           break into little equal chunks
        #                 are these chunks the same size?
        #                 cut each 'state' to the length of the shortest?
        #           make a dataset out of the chunks along with labels (chunk_x, y)
        #           store parameters for these chunks
        # 
        #       for each little chunk dataset, train a classifier
        #           svd the dataset, select reasonable number of dimensions
        #           create a kd tree
        # 
        # in the end we want 
        # {1: [[PCAKNeighborClassifier, tstart, tend], [PCAKNeighborClassifier, tstart, tend]... ],
        #  2: ...
        #  3: ...
        #  }
        pass
    
    def classify(self, data_mat, time_mat):
        pass


class EventDetector:
    def __init__(self):
        # subscribe to pressure topics
        # subscribe to behavior markers
        pass
       
    def callback(self, msg):
        # when we have enough data for a period in a behavior, classify..
        pass

def construct_pressure_marker_message(data_dict, topic, base_color=np.matrix([1.,0, 0, 1.]).T):
    print 'creating markers...'
    #record_number = 0
    STATE_SEPARATION_DIST = .4
    points_ll = []
    colors_ll = []
    pressures_l = []
    
    times_dict = {}
    for i in range(len(data_dict.keys())):
        times_dict[i] = []
        for record_number in range(len(data_dict[i][topic])):
            time_start = data_dict[0][topic][record_number]['t'][0]
            times_dict[i].append(data_dict[i][topic][record_number]['t'] - time_start)

    state_time_offsets = {}
    state_time_offsets[-1] = {'duration':0, 'offset':0}
    #state_time_offsets[0] = 0
    for state in range(len(times_dict.keys())):
        durations = [state_times[-1] - state_times[0] for state_times in times_dict[state]]
        duration_state = np.max(durations)
        state_time_offsets[state] = {'duration': duration_state,
                                     'offset': state_time_offsets[state-1]['offset'] + state_time_offsets[state-1]['duration'] + STATE_SEPARATION_DIST}
        print 'state', state, 'offset', state_time_offsets[state]['offset'], 'duration', state_time_offsets[state]['duration']
    
    for record_number in range(len(data_dict[0][topic])):
        print record_number, len(data_dict[0][topic]), len(data_dict[1][topic]), len(data_dict[2][topic])
        pressure_mat_l = np.column_stack([data_dict[i][topic][record_number]['left' ] for i in range(len(data_dict.keys()))])
        pressure_mat_r = np.column_stack([data_dict[i][topic][record_number]['right'] for i in range(len(data_dict.keys()))])
        pressure_mat = np.row_stack((pressure_mat_l, pressure_mat_r))
        pressure_mat = (pressure_mat - pressure_mat[:,0])

        times_l = []
        for i in range(len(data_dict.keys())):
            #times_l.append(np.matrix(state_time_offsets[i]['offset'] + data_dict[i][topic][record_number]['t'] - data_dict[0][topic][record_number]['t'][0]))
            curr_times = data_dict[i][topic][record_number]['t']
            curr_times = curr_times - curr_times[0]
            times_l.append(np.matrix(curr_times + state_time_offsets[i]['offset']))
            #times_l.append(np.matrix( - data_dict[0][topic][record_number]['t'][0]))
        times_m = np.column_stack(times_l)
    
        X, Y = np.meshgrid(range(pressure_mat.shape[0]), range(pressure_mat.shape[1]))
        x_multiplier = 1/15.
        #y_multiplier = 1/10.
        x_size = pressure_mat.shape[0] * x_multiplier
        #y_size = pressure_mat.shape[1] * y_multiplier

        X = np.matrix(X * x_multiplier) + x_size*record_number + (record_number*x_size / 3.)
        #Y0 = np.matrix(Y * y_multiplier)
        Y = (np.matrix(np.ones((pressure_mat.shape[0], 1))) * times_m).T
        Z = np.matrix(np.zeros(pressure_mat.shape))

        points = np.row_stack((X.reshape(1, X.shape[0] * X.shape[1]),
                               Y.reshape(1, Y.shape[0] * Y.shape[1]),
                               Z.reshape(1, Z.shape[0] * Z.shape[1])))
        colors = np.matrix(np.zeros((4, pressure_mat.shape[0]*pressure_mat.shape[1])))
        pressures_l.append(pressure_mat.T.reshape((1,pressure_mat.shape[1] * pressure_mat.shape[0])))

        points_ll.append(points)
        colors_ll.append(colors)

    pressures = np.column_stack(pressures_l)
    all_points = np.column_stack(points_ll)
    colors_mat = np.column_stack(colors_ll)

    #pdb.set_trace()
    max_pval = np.max(pressures)
    min_pval = np.min(pressures)
    print 'min %d max %d' % (min_pval, max_pval)

    #base_color        = np.matrix([1.,0, 0, 1.]).T
    colors_mat[0:4,:] = base_color * (np.abs(pressures) / 100.)
    marker      = viz.list_marker(all_points, colors_mat, [.05, .05, .05], 'points', 'pressure_viz')
    #point_cloud = ru.np_to_colored_pointcloud(all_points, np.matrix(pressures)/100. + min_pval, 'pressure_viz')
    return marker


class DisplayDataWithRviz:

    def __init__(self):
        rospy.init_node('display_pressure_with_rviz')
        self.succ_marker = rospy.Publisher('succ_marker', vm.Marker)
        self.fail_marker = rospy.Publisher('fail_marker', vm.Marker)

        self.succ_pc_pub = rospy.Publisher('succ_pc', sm.PointCloud)
        self.fail_pc_pub = rospy.Publisher('fail_pc', sm.PointCloud)

    def display(self, succ_pickle, fail_pickle):
        # load in pickle
        print 'loading...'
        successes = ut.load_pickle(succ_pickle)
        failures = ut.load_pickle(fail_pickle)

        print 'Enter the topic number:'
        for i, k in enumerate(successes[0].keys()):
            print i, k
        topic = successes[0].keys()[int(raw_input())]

        red = np.matrix([1.,0, 0, 1.]).T
        green = np.matrix([0.,1., 0, 1.]).T
        #succ_marker = construct_pressure_marker_message(successes, topic, green)
        fail_marker = construct_pressure_marker_message(failures, topic, red)
        print 'publishing...'
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            #self.succ_marker.publish(succ_marker)
            self.fail_marker.publish(fail_marker)
            r.sleep()

        #def list_marker(points, colors, scale, mtype, mframe, duration=10.0, m_id=0):
        #data_dict[i][ptop].append({'left': left_f, 'right': right_f, 't': ptimes})

#data_dict [state number] [topic] [trial number] ['t' 'left' 'right']
def average_reading_over_trials(data_dict):
    #Construct list of list of matrices indexing [state][trial number] => contact information for both fingers
    contact_info = {}
    for state in data_dict.keys():
        contact_info[state] = []
        for trial in data_dict[state][topic]:
            contact_info[state].append(np.row_stack((trial['left'], trial['right'])))
    
    ret_dict = {}
    #shorten the trials to be the length of the shortest trial
    for state in contact_info.keys():
        shortest_length = np.min(trial.shape[1] for trial in contact_info[state])
        trimmed_mats    = [trial[:,shortest_length] for trial in contact_info[state]]
        avg_reading     = np.sum(np.concatenate([np.reshape(trial, (trial.shape[0], trial.shape[1], 1)) for trial in trimmed_mats], 2)) / len(trimmed_mats)
        div_point = avg_reading.shape[0]
        assert(div_point == 22)
        ret_dict[state] = {topic: [{'t': contact_info[state][0][:shortest_length],
                                    'left': avg_reading[:div_point, :],
                                    'right': avg_reading[div_point:, :]}]
    return ret_dict

#Debug this!
class DiffDisplay:

    def __init__(self):
        self.succ_marker = rospy.Publisher('succ_avg', vm.Marker)
        self.fail_marker = rospy.Publisher('fail_avg', vm.Marker)

    def display(self, succ_pickle, fail_pickle):
        # load in pickle
        print 'loading...'
        successes = ut.load_pickle(succ_pickle)
        failures = ut.load_pickle(fail_pickle)

        topics = ['/pressure/l_gripper_motor', '/pressure/r_gripper_motor']
        topic = topics[0]

        red = np.matrix([1.,0, 0, 1.]).T
        green = np.matrix([0.,1., 0, 1.]).T
        #data_dict [state number] [topic] [trial number] ['t' 'left' 'right']
        succ_marker = construct_pressure_marker_message(average_reading_over_trials(successes), topic, green)
        fail_marker = construct_pressure_marker_message(average_reading_over_trials(failures), topic, red)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.succ_marker.publish(succ_marker)
            self.fail_marker.publish(fail_marker)
            r.sleep()



if __name__ == '__main__':
    import sys

    if 'preprocess' == sys.argv[1]:
        #print 'Loading success bags..'
        #succ_dict = success_failure_classification_preprocess(sys.argv[2])
        #print 'Saving success dict.'
        #ut.save_pickle(succ_dict, '%s/success_data.pkl' % sys.argv[2])

        print 'Loading failure bags..'
        fail_dict = success_failure_classification_preprocess(sys.argv[3])
        print 'Saving failure dict.'
        ut.save_pickle(fail_dict, '%s/failure_data.pkl' % sys.argv[3])

        print 'Done!'

    if 'learn' == sys.argv[1]:
        classifier = TimeSeriesClassifier()
        classifier.learn_model(sys.argv[2], sys.argv[3])

    #Debug this to display failure cases
    if 'display' == sys.argv[1]:
        d = DisplayDataWithRviz()
        #data_dict [state number] [topic] [trial number] ['t' 'left' 'right']
        d.display(sys.argv[2], sys.argv[3])

    #Dbug this so that it displays the average value, and the diff between the averages.
    if 'diff' == sys.argv[1]:
        d = DiffDisplay()
        #data_dict [state number] [topic] [trial number] ['t' 'left' 'right']
        d.diff(sys.argv[2], sys.argv[3])




        #for d in durations:
        #    print '%.3f' % d

        #all_start_times = [tarr[0] for tarr in times_dict[state]]
        #state_time_offsets[state] = np.max(all_start_times)
        #all_end_times = [tarr[-1] for tarr in times_dict[state]]
        #state_time_offsets[state+1] = np.max(all_end_times)

        #print 'Enter the state number len(successes) %d len(failures) %d:' % (len(successes), len(failures))
        #state_numb = int(raw_input())

        #print 'left (0)? right (1)?'
        #fnumb = int(raw_input())
        #if fnumb == 0:
        #    finger = 'left'
        #elif fnumb == 1:
        #    finger = 'right'

        #print 'record number? (%d)' % len(successes[state_numb][topic])
        #record_number = int(raw_input())


        #threshold = 3000
        #pressures[np.where(pressures < threshold)] = 0
        #pressures[np.where(pressures >= threshold)] = 1.


            #times_list = []
            #tstart = None
            #for i in range(len(successes.keys())):
            #    state_i_times = np.matrix(successes[i][topic][record_number]['t'])
            #    if tstart == None:
            #        tstart = state_i_times[0,0]

            #    state_i_times = state_i_times - tstart
            #    tstart = state_i_times[:,-1] + 1.

            #    times_list.append()


