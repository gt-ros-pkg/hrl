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
import hai_sandbox.dimreduce as dimreduce
import sensor_msgs.msg as sm
import scipy.spatial as sp
import pdb

def select_time(data, time_rec, time_start, time_end):
    #pdb.set_trace()
    return data[:, np.where(np.multiply(time_rec >= time_start, time_rec < time_end))[0]]

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


def break_record_matrices_into_chunks(data_dict, segmented_matrices, minimal_trial_lengths, chunk_times, topic):
    #For each state
    data_sets = {}
    for state in range(len(data_dict.keys())):
        data_sets[state] = {}
        time_starts = np.arange(0, minimal_trial_lengths[state], chunk_times[state])
        time_ends   = np.arange(chunk_times[state], minimal_trial_lengths[state] + chunk_times[state], chunk_times[state])

        #Go over all the records
        for record_number in range(len(data_dict[state][topic])):
            time_rec = segmented_matrices[record_number]['t'][state]
            time_rec = time_rec - time_rec[0]
            #Break them up into little chunks
            for tidx in range(len(time_starts)):
                if not data_sets[state].has_key(tidx):
                    data_sets[state][tidx] = {}
                    data_sets[state][tidx]['data'] = []
                    data_sets[state][tidx]['time'] = [time_starts[tidx], time_ends[tidx]]

                #pdb.set_trace()
                data_chunk = select_time(segmented_matrices[record_number]['mat'][state], time_rec, 
                                            time_starts[tidx], time_ends[tidx])
                data_sets[state][tidx]['data'].append(data_chunk)
    return data_sets



class TimeSeriesClassifier:

    def __init__(self):
        pass

    def preprocess_pickles(self, successes, failures, topic, segment_length):
        #Break matrices into segments based on state
        # list of list of 44xN matrices
        success_matrices_segmented_by_state = construct_list_of_segmented_matrices_from_trial_recording(successes, topic)
        failure_matrices_segmented_by_state = construct_list_of_segmented_matrices_from_trial_recording(failures, topic)

        #Calculate how long each chunk needs to be
        success_trial_durations = find_trial_durations(successes, topic) # trial_durations[state][trial number]
        failure_trial_durations = find_trial_durations(failures, topic) # trial_durations[state][trial number]
        #for state in range(len(times_dict.keys())):
        #    durations = [state_times[-1] - state_times[0] for state_times in times_dict[state]]

        #pdb.set_trace()
        minimal_trial_lengths = [np.min(np.min(success_trial_durations[state]), np.min(failure_trial_durations[state])) \
                for state in range(len(success_trial_durations.keys()))]
        chunk_times = [length/np.floor(length/segment_length) for length in minimal_trial_lengths]

        #make little chunks out of the matrices of pressure readings
        success_data_sets = break_record_matrices_into_chunks(successes, success_matrices_segmented_by_state, \
                minimal_trial_lengths, chunk_times, topic)
        failure_data_sets = break_record_matrices_into_chunks(failures, failure_matrices_segmented_by_state, \
                minimal_trial_lengths, chunk_times, topic)

        #Make sure the little chunks are of the same dimension across positive and negative examples
        chunk_dim = {}
        for state in range(len(successes.keys())):
            chunk_dim[state] = {}
            for chunk_idx in range(len(success_data_sets[state])):
                #figure minimum chunk lengths in array size
                chunk_length_successes = [success_data_sets[state][chunk_idx]['data'][chunk_record].shape[1] \
                        for chunk_record in range(len(success_data_sets[state][chunk_idx]['data']))]
                chunk_length_failures  = [failure_data_sets[state][chunk_idx]['data'][chunk_record].shape[1] \
                        for chunk_record in range(len(failure_data_sets[state][chunk_idx]['data']))]
                #pdb.set_trace()
                shortest_chunk_length = np.min([np.min(chunk_length_successes), np.min(chunk_length_failures)])
                chunk_dim[state][chunk_idx] = shortest_chunk_length
                #shorten data to shortest_chunk_length
                #if state == 0 and chunk_idx == 2:
                #    pdb.set_trace()

        for state in range(len(successes.keys())):
            for chunk_idx in range(len(success_data_sets[state])):                
                for dataset_idx, data_sets in enumerate([success_data_sets, failure_data_sets]):
                    for chunk_record in range(len(data_sets[state][chunk_idx]['data'])):
                        shortest_chunk_length = chunk_dim[state][chunk_idx]
                        data_sets[state][chunk_idx]['data'][chunk_record] = \
                                data_sets[state][chunk_idx]['data'][chunk_record][:,:shortest_chunk_length]
                        #if state == 0 and chunk_idx == 2:
                        #    #pdb.set_trace()
                        #    print data_sets[state][chunk_idx].__class__, 'len(data_sets[state][chunk_idx])', len(data_sets[state][chunk_idx]), 'dataset idx', dataset_idx, 'shortest_chunk_length', shortest_chunk_length, 'shape', data_sets[state][chunk_idx]['data'][chunk_record].shape

        chunk_params = {'chunk_dim': chunk_dim,
                        'trial_lengths': minimal_trial_lengths,
                        'chunk_times': chunk_times,
                        'topic': topic}
        return success_data_sets, failure_data_sets, chunk_params
                                                      

    def save_models(self, name='timeseries_pca_model.pkl'):
        print 'saving models'
        ut.save_pickle(self.models, name)

    def load_models(self, name='timeseries_pca_model.pkl'):
        print 'loading models'
        self.models = ut.load_pickle(name)

    def create_matrix_from_chunked_datasets(self, datasets):
        mat_set = {}
        for state in range(len(datasets.keys())):
            mat_set[state] = {}
            for chunk_idx in range(len(datasets[state])):
                records_l = []
                for chunk_record in range(len(datasets[state][chunk_idx]['data'])):
                    a = datasets[state][chunk_idx]['data'][chunk_record]
                    records_l.append(np.reshape(a, (a.shape[0]*a.shape[1], 1)))

                mat_set[state][chunk_idx] = {}
                mat_set[state][chunk_idx]['data'] = np.column_stack(records_l) 
                mat_set[state][chunk_idx]['time'] = datasets[state][chunk_idx]['time']
        return mat_set

    def preprocess_individual_pickle(self, apickle):
        data = ut.load_pickle(apickle)
        models, chunk_params = self.models
        #break pickle into chunks given model.
        #chunks of equivalent time, chunks of equivalent dimensions
        data_segmented = construct_list_of_segmented_matrices_from_trial_recording(data)

        # 1) break into time chunks
        chunked_data = break_record_matrices_into_chunks(data, data_segmented, \
                chunk_params['trial_lengths'], chunk_params['chunk_times'], chunk_params['topic'])

        # 2) shorten into appropriate dimensions
        for state in range(len(models)):
            for chunk_idx in range(len(models[state])):
                for chunk_record in range(len(chunked_data[state][chunk_idx]['data'])):
                    chunk_length = chunk_params['chunk_dim'][state][chunk_idx]
                    chunked_data[state][chunk_idx]['data'][chunk_record] =\
                            chunked_data[state][chunk_idx]['data'][chunk_record][:, :chunk_length]

        return chunked_data

    def classify_pickle(self, apickle):
        # a pickle can have multiple records...
        chunked_data = self.preprocess_individual_pickle(apickle)
        #mat_set = self.create_matrix_from_chunked_datasets(chunked_data)
        models = self.models[0]

        for record_idx in chunked_data[0][0]['data']:
            for state in range(len(models)):
                for chunk_idx in range(len(models[state])):
                    x_mat = chunked_data[state][chunk_idx]['data'][chunk_record]
                    x_vec = np.reshape((a.shape[0] * a.shape[1], 1))
                    projected_x = (models[state][chunk_idx]['project'].T * x_vec).T.A1
                    nearest_neighbor = models[state][chunk_idx]['tree'].query(projected_x, 1)
                    ???
                    models

    def create_model(self, succ_pickle, fail_pickle):
        topic = '/pressure/l_gripper_motor'
        SEGMENT_LENGTH = 1.0
        VARIANCE_KEEP = .9

        # load in pickle
        print 'loading pickles'
        successes = ut.load_pickle(succ_pickle)
        failures = ut.load_pickle(fail_pickle)

        #chop data set into little chunks
        # data_sets[state][chunk_idx]['data', 'time'][chunk_record]
        print 'preprocess pickles'
        success_data_sets, failure_data_sets, chunk_params = self.preprocess_pickles(successes, \
                failures, topic, SEGMENT_LENGTH)

        # turn each set of records into a matrix
        combined_sets = {}
        for dset_name, datasets in zip(['success', 'failure'], [success_data_sets, failure_data_sets]):
            #merge the two matrices from mat_set
            mat_set = self.create_matrix_from_chunked_datasets(datasets)
            for state in range(len(datasets.keys())):
                combined_sets[state] = {}
                for chunk_idx in range(len(datasets[state])):
                    combined_sets[state][chunk_idx] = {}
                    combined_sets[state][chunk_idx][dset_name] = mat_set[state][chunk_idx]['data']
                    combined_sets[state][chunk_idx]['time'] = mat_set[state][chunk_idx]['time']
        
        #for dset_name, datasets in zip(['success', 'failure'], [success_data_sets, failure_data_sets]):
        #    for state in range(len(datasets.keys())):
        #        #print '>> state', state, 'num chunks', len(datasets[state])
        #        if not combined_sets.has_key(state):
        #            combined_sets[state] = {}
        #        for chunk_idx in range(len(datasets[state])):
        #            records_l = []
        #            for chunk_record in range(len(datasets[state][chunk_idx]['data'])):
        #                a = datasets[state][chunk_idx]['data'][chunk_record]
        #                records_l.append(np.reshape(a, (a.shape[0]*a.shape[1], 1)))

        #            if not combined_sets[state].has_key(chunk_idx):
        #                combined_sets[state][chunk_idx] = {}

        #            combined_sets[state][chunk_idx][dset_name] = np.column_stack(records_l) 
        #            combined_sets[state][chunk_idx]['time'] = datasets[state][chunk_idx]['time']

        # run PCA over the entire set
        models = {}
        for state in range(len(combined_sets.keys())):
            models[state] = []
            for chunk_idx in range(len(combined_sets[state])):
                print 'building model for state', state, 'chunk idx', chunk_idx
                #pdb.set_trace()
                data_chunk = np.column_stack((combined_sets[state][chunk_idx]['success'], \
                                           combined_sets[state][chunk_idx]['failure']))
                num_pos = combined_sets[state][chunk_idx]['success'].shape[1]
                num_neg = combined_sets[state][chunk_idx]['failure'].shape[1]
                labels = np.column_stack(np.matrix(np.ones((1, num_pos))), np.matrix(np.zeros((1, num_neg))))

                projection_basis = dimreduce.pca_vectors(data_chunk, VARIANCE_KEEP)
                reduced_data = projection_basis.T * data_chunk
                models[state].append({'time':    combined_sets[state][chunk_idx]['time'],
                                      'project': projection_basis,
                                      'reduced': reduced_data,
                                      'labels':  labels,
                                      'data':    data_chunk,
                                      'tree':    sp.KDTree(np.array(reduced_data.T))})

        self.models = {'models':models, 
                       'chunk_params': chunk_params}
                

        #def segment_pressure_dataset(pdict):
        #    topics = ['/pressure/l_gripper_motor', '/pressure/r_gripper_motor']
        #    for topic in topics:
        #        #find longest record in terms of number of messages
        #        shortest_left = 0
        #        shortest_right = 0
        #        for record in pdict[topic]:
        #            shortest_left = min(record['left'].shape[1], shortest_left)
        #            shortest_right = min(record['right'].shape[1], longest_right)
        #            #pdb.set_trace()
        #            #record['t'][0]
        #        ##break up existing records
        #        #for record in pdict[topic]:
        #        #    record['left']
        ## for each state
        #for state_numb in range(len(successes.keys())):
        #    #data_dict[i][ptop].append({'left': left_f, 'right': right_f, 't': ptimes})
        #    segment_pressure_dataset(successes[state_numb])
        #    failures[state_numb]
        #    pdb.set_trace()

                



        #       for each record of success and failure
        #           break into little equal chunks based on time in trial
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
        #pass
    
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

def zero_out_time_in_trials(data_dict, topic):
    #print 'Finding times...'
    #pdb.set_trace()
    times_dict = {} # times_dict[state][ list of durations ]
    for state in range(len(data_dict.keys())):
        times_dict[state] = []
        for record_number in range(len(data_dict[state][topic])):
            time_start = data_dict[0][topic][record_number]['t'][0]
            #pdb.set_trace()
            times_dict[state].append(data_dict[state][topic][record_number]['t'] - time_start)
    return times_dict

def find_trial_durations(data_dict, topic):
    times_dict = {} # times_dict[state][ list of durations ]
    for state in range(len(data_dict.keys())):
        times_dict[state] = []
        for record_number in range(len(data_dict[state][topic])):
            time_start = data_dict[state][topic][record_number]['t'][0]
            time_end = data_dict[state][topic][record_number]['t'][-1]
            times_dict[state].append(time_end - time_start)
    return times_dict

def construct_pressure_marker_message(data_dict, topic, base_color=np.matrix([1.,0, 0, 1.]).T):
    #record_number = 0
    STATE_SEPARATION_DIST = .4
    points_ll = []
    colors_ll = []
    pressures_l = []
   
    #Record the duration of each trial
    times_dict = zero_out_time_in_trials(data_dict, topic)
    #print 'Finding times...'
    #times_dict = {} # times_dict[state][ list of durations ]
    #for i in range(len(data_dict.keys())):
    #    times_dict[i] = []
    #    for record_number in range(len(data_dict[i][topic])):
    #        time_start = data_dict[0][topic][record_number]['t'][0]
    #        #pdb.set_trace()
    #        times_dict[i].append(data_dict[i][topic][record_number]['t'] - time_start)

    #Use the durations to figure out offsets
    state_time_offsets = {}
    state_time_offsets[-1] = {'duration':0, 'offset':0}
    #state_time_offsets[0] = 0
    for state in range(len(times_dict.keys())):
        durations = [state_times[-1] - state_times[0] for state_times in times_dict[state]]
        duration_state = np.max(durations)
        state_time_offsets[state] = {'duration': duration_state,
                                     'offset': state_time_offsets[state-1]['offset'] + state_time_offsets[state-1]['duration'] + STATE_SEPARATION_DIST}
        print 'state', state, 'offset', state_time_offsets[state]['offset'], 'duration', state_time_offsets[state]['duration']

    #Create corrected timelines
    times_m_list = []
    for record_number in range(len(data_dict[0][topic])):
        #range(len(data_dict[0][topic])):
        #print record_number, len(data_dict[0][topic]), len(data_dict[1][topic]), len(data_dict[2][topic])
        #pressure_mat_l = np.column_stack([data_dict[i][topic][record_number]['left' ] for i in range(len(data_dict.keys()))])
        #pressure_mat_r = np.column_stack([data_dict[i][topic][record_number]['right'] for i in range(len(data_dict.keys()))])
        #pressure_mat = np.row_stack((pressure_mat_l, pressure_mat_r))
        #pdb.set_trace()

        #For each state figure out the time offset & store in times_l
        times_l = []
        for i in range(len(data_dict.keys())):
            #times_l.append(np.matrix(state_time_offsets[i]['offset'] + data_dict[i][topic][record_number]['t'] - data_dict[0][topic][record_number]['t'][0]))
            curr_times = data_dict[i][topic][record_number]['t']
            curr_times = curr_times - curr_times[0]
            times_l.append(np.matrix(curr_times + state_time_offsets[i]['offset']))
            #times_l.append(np.matrix( - data_dict[0][topic][record_number]['t'][0]))
        #Stack times_l to form times_m
        times_m = np.column_stack(times_l)
        times_m_list.append(times_m)

    
    ##Create segmented matrices
    #for record_number, list_pressure_mat \
    #        in enumerate(construct_list_of_segmented_matrices_from_trial_recording(data_dict, topic)):
    #    pressure_mat = np.column_stack(list_pressure_mat)
    #    pressure_mat = (pressure_mat - pressure_mat[:,0])

    #    X, Y = np.meshgrid(range(pressure_mat.shape[0]), range(pressure_mat.shape[1]))
    #    x_multiplier = 1/15.
    #    x_size = pressure_mat.shape[0] * x_multiplier

    #    #y_multiplier = 1/10.
    #    #y_size = pressure_mat.shape[1] * y_multiplier

    #    #Y0 np.matrix(np.ones((pressure_mat.shape[0], 1)))= np.matrix(Y * y_multiplier)
    #    X = np.matrix(X * x_multiplier) + x_size*record_number + (record_number*x_size / 3.)
    #    Y = (np.matrix(np.ones((pressure_mat.shape[0], 1))) * times_m).T
    #    Z = np.matrix(np.zeros(pressure_mat.shape))

    #    points = np.row_stack((X.reshape(1, X.shape[0] * X.shape[1]),
    #                           Y.reshape(1, Y.shape[0] * Y.shape[1]),
    #                           Z.reshape(1, Z.shape[0] * Z.shape[1])))
    #    colors = np.matrix(np.zeros((4, pressure_mat.shape[0]*pressure_mat.shape[1])))
    #    pressures_l.append(pressure_mat.T.reshape((1,pressure_mat.shape[1] * pressure_mat.shape[0])))
    #    points_ll.append(points)
    #    colors_ll.append(colors)

    print 'constructing segmented matrices...'
    pressure_mats = []
    for lp in construct_list_of_segmented_matrices_from_trial_recording(data_dict, topic):
        p = np.column_stack(lp) 
        p = p - p[:,0]

        #min_p = np.min(p)
        #range_p = float((np.max(p) - min_p))
        #p = ((p - min_p) / range_p) * 1000.

        pressure_mats.append(p)

    print 'creating colored points...'
    pressures, all_points, colors_mat = create_colored_3d_points_from_matrices(pressure_mats, times_m_list)

    #pressures = np.column_stack(pressures_l)
    #all_points = np.column_stack(points_ll)
    #colors_mat = np.column_stack(colors_ll)
    #pdb.set_trace()
    #max_pval = np.max(pressures)
    #min_pval = np.min(pressures)
    #print 'min pressure reading %d max %d' % (min_pval, max_pval)

    #print 'making colors...'
    #base_color        = np.matrix([1.,0, 0, 1.]).T
    #colors_mat[0:4,:] = base_color * (np.abs(pressures) / 100.)
    #print 'creating marker message'
    #marker      = viz.list_marker(all_points, colors_mat, [.05, .05, .05], 'points', 'pressure_viz')
    #return marker, point_cloud
    print 'creating pointcloud message'
    #point_cloud = ru.np_to_colored_pointcloud(all_points, np.matrix(pressures) + min_pval, 'pressure_viz')
    point_cloud = ru.np_to_colored_pointcloud(all_points, np.matrix(pressures), 'pressure_viz')
    return point_cloud


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
        print 'construct_pressure_marker_message(successes, topic, green)'
        #succ_marker, succ_pc = construct_pressure_marker_message(successes, topic, green)
        succ_pc = construct_pressure_marker_message(successes, topic, green)
        print 'construct_pressure_marker_message(failures, topic, red)'
        #fail_marker, fail_pc = construct_pressure_marker_message(failures, topic, red)
        fail_pc = construct_pressure_marker_message(failures, topic, red)
        print 'publishing...'
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            #self.succ_marker.publish(succ_marker)
            #self.fail_marker.publish(fail_marker)
            self.succ_pc_pub.publish(succ_pc)
            self.fail_pc_pub.publish(fail_pc)
            r.sleep()

        #def list_marker(points, colors, scale, mtype, mframe, duration=10.0, m_id=0):
        #data_dict[i][ptop].append({'left': left_f, 'right': right_f, 't': ptimes})

##
# Create matrices split based on state
#
# @param data_dict [state number] [topic] [trial number] ['t' 'left' 'right']
# @param topic string
# @return segmented_matrices[record_number][state_number]['t'] =>   1xN array          
#                                                        ['mat'] => 44xN mat
def construct_list_of_segmented_matrices_from_trial_recording(data_dict, topic):
    segmented_matrices = []
    #pdb.set_trace()
    for record_number in range(len(data_dict[0][topic])):
        segmented_matrix = []
        trecs = []
        for state in range(len(data_dict.keys())):
            segmented_matrix.append(np.row_stack((data_dict[state][topic][record_number]['left'], 
                                                  data_dict[state][topic][record_number]['right'])))
            trecs.append(data_dict[state][topic][record_number]['t'])
        segmented_matrices.append({'mat': segmented_matrix,
                                   't': trecs})
        #segmented_matrices.append(segmented_matrix)
    return segmented_matrices

#def construct_marker_message_from_list_of_segmented_matrices(segmented_matrix, slot_number, column_index):
def create_colored_3d_points_from_matrices(matrices, index_list):
    points3d_l = []
    colors_ll = []
    mat_l = []
    X_MULTIPLIER = 1/15.

    for i, mat in enumerate(matrices):
        X, Y = np.meshgrid(range(mat.shape[0]), range(mat.shape[1]))
        x_size = mat.shape[0] * X_MULTIPLIER
        X = np.matrix(X * X_MULTIPLIER) + x_size * i + (i * x_size / 3.)
        #Y = (np.matrix(np.ones((mat.shape[0], 1))) * times_m).T
        Y = (np.matrix(np.ones((mat.shape[0], 1))) * index_list[i]).T
        #pdb.set_trace()
        Z = np.matrix(np.zeros(mat.shape)).T

        points = np.row_stack((X.reshape(1, X.shape[0] * X.shape[1]),
                               Y.reshape(1, Y.shape[0] * Y.shape[1]),
                               Z.reshape(1, Z.shape[0] * Z.shape[1])))
        colors = np.matrix(np.zeros((4, mat.shape[0]*mat.shape[1])))
        mat_l.append(mat.T.reshape((1,mat.shape[1] * mat.shape[0])))
        points3d_l.append(points)
        colors_ll.append(colors)

    all_mats = np.column_stack(mat_l)
    all_points = np.column_stack(points3d_l)
    all_colors = np.column_stack(colors_ll)
    return all_mats, all_points, all_colors

#data_dict [state number] [topic] [trial number] ['t' 'left' 'right']
def average_reading_over_trials(data_dict, topic):
    #Construct list of list of matrices indexing [state][trial number] => contact information for both fingers
    contact_info = {}
    for state in data_dict.keys():
        contact_info[state] = []
        for trial in data_dict[state][topic]:
            contact_info[state].append(np.row_stack((trial['left'], trial['right'])))
    
    ret_dict = {}
    #shorten the trials to be the length of the shortest trial
    for state in contact_info.keys():
        shortest_length = np.min([trial.shape[1] for trial in contact_info[state]])
        trimmed_mats    = [trial[:,:shortest_length] for trial in contact_info[state]]
        avg_reading     = np.matrix(np.sum(np.concatenate([np.reshape(np.array(trial), (trial.shape[0], trial.shape[1], 1)) for trial in trimmed_mats], 2), 2) / len(trimmed_mats))
        div_point = avg_reading.shape[0]/2.
        assert(div_point == 22)
        ret_dict[state] = {topic: [{'t':     data_dict[state][topic][0]['t'][:shortest_length] ,#contact_info[state][0][:shortest_length],
                                    'left':  avg_reading[:div_point,:],
                                    'right': avg_reading[div_point:,:]}] }
    return ret_dict


def subtract_records(recorda, recordb, topic):
    ret_dict = {}
    for state in recorda.keys():
        shortest_length = min(recorda[state][topic][0]['left'].shape[1], recordb[state][topic][0]['left'].shape[1])
        ret_dict[state] = {topic: [{
            't':     recorda[state][topic][0]['t'][:shortest_length],
            'left':  np.abs(recorda[state][topic][0]['left'][:,:shortest_length] - recordb[state][topic][0]['left'][:,:shortest_length]),
            'right': np.abs(recorda[state][topic][0]['right'][:,:shortest_length] - recordb[state][topic][0]['right'][:,:shortest_length])
            }]}
    return ret_dict


#Debug this!
class DiffDisplay:

    def __init__(self):
        rospy.init_node('diff_display')
        self.fail_marker = rospy.Publisher('diff_fail_avg', vm.Marker)
        self.fail_pc_pub = rospy.Publisher('diff_fail_pc', sm.PointCloud)

        self.succ_marker = rospy.Publisher('diff_succ_avg', vm.Marker)
        self.succ_pc_pub = rospy.Publisher('diff_succ_pc', sm.PointCloud)

        self.diff_marker = rospy.Publisher('diff_avg', vm.Marker)
        self.diff_pc_pub = rospy.Publisher('diff_pc', sm.PointCloud)

    def display(self, succ_pickle, fail_pickle):
        # load in pickle
        print 'loading...'
        successes = ut.load_pickle(succ_pickle)
        failures = ut.load_pickle(fail_pickle)

        topics = ['/pressure/l_gripper_motor', '/pressure/r_gripper_motor']
        topic = topics[0]

        red = np.matrix([1., 0, 0, 1.]).T
        green = np.matrix([0., 1., 0, 1.]).T
        blue = np.matrix([0., 0, 1., 1.]).T

        #data_dict [state number] [topic] [trial number] ['t' 'left' 'right']
        succ_avg = average_reading_over_trials(successes, topic)
        fail_avg = average_reading_over_trials(failures, topic)
        diff_avg = subtract_records(succ_avg, fail_avg, topic)

        succ_marker, succ_pc = construct_pressure_marker_message(succ_avg, topic, green)
        fail_marker, fail_pc = construct_pressure_marker_message(fail_avg, topic, red)
        diff_marker, diff_pc = construct_pressure_marker_message(diff_avg, topic, blue)

        r = rospy.Rate(10)
        print 'publishing...'
        while not rospy.is_shutdown():
            self.succ_marker.publish(succ_marker)
            self.fail_marker.publish(fail_marker)

            self.succ_pc_pub.publish(succ_pc)
            self.fail_pc_pub.publish(fail_pc)

            self.diff_marker.publish(diff_marker)
            self.diff_pc_pub.publish(diff_pc)
            r.sleep()


if __name__ == '__main__':
    import sys

    if 'preprocess' == sys.argv[1]:
        print 'Loading success bags..'
        succ_dict = success_failure_classification_preprocess(sys.argv[2])
        print 'Saving success dict.'
        ut.save_pickle(succ_dict, '%s/success_data.pkl' % sys.argv[2])

        print 'Loading failure bags..'
        fail_dict = success_failure_classification_preprocess(sys.argv[3])
        print 'Saving failure dict.'
        ut.save_pickle(fail_dict, '%s/failure_data.pkl' % sys.argv[3])

        print 'Done!'

    if 'learn' == sys.argv[1]:
        classifier = TimeSeriesClassifier()
        classifier.create_model(sys.argv[2], sys.argv[3])
        classifier.save_models()

    #Debug this to display failure cases
    if 'display' == sys.argv[1]:
        d = DisplayDataWithRviz()
        #data_dict [state number] [topic] [trial number] ['t' 'left' 'right']
        d.display(sys.argv[2], sys.argv[3])

    #Dbug this so that it displays the average value, and the diff between the averages.
    if 'diff' == sys.argv[1]:
        d = DiffDisplay()
        #data_dict [state number] [topic] [trial number] ['t' 'left' 'right']
        d.display(sys.argv[2], sys.argv[3])


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


