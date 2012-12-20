
import sys, os
import numpy as np, math

import scipy.stats as ss

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import hrl_lib.util as ut



class ComputeResultStatistics():
    def __init__(self):
        self.root_path = None
        pass

    def traverse_directory(self, root_path, fn_list):
        self.root_path = root_path

        for fn in fn_list:
            fn('start')

        for root, dirs, files in os.walk(root_path):
            for d in dirs:
                full_path = os.path.join(root, d)
                print full_path
                for fn in fn_list:
                    fn(full_path)

        for fn in fn_list:
            fn('end')

    def get_sum_taxel_hist(self, full_path):
        if full_path == 'start':
            self.taxel_hist = None
            return

        if full_path == 'end':
            d = {}
            d['taxel_hist'] = self.taxel_hist.tolist()
            ut.save_pickle(d, self.root_path + '/taxel_histogram.pkl')
            print "here's taxel hist :\n", self.taxel_hist

        full_path = full_path.rstrip('/')
        res_dict = ut.load_pickle(full_path+'/reach_from_reach_in_dy_0.00_retry_right=to_goal_1_log.pkl')
        trial = full_path.split('/')[-1]

        if self.taxel_hist == None:
            try:
                self.taxel_hist = np.array([0]*len(res_dict['taxel_hist']))
            except:
                print "no taxel_hist in this logging files"
        if res_dict is None:
            pass
        else: 
            self.taxel_hist = self.taxel_hist + res_dict['taxel_hist']

    def combine_controller_rates(self, full_path):
        if full_path == 'start':
            self.controller_rate = []
            return

        if full_path == 'end':
            d = {}
            d['all_controller_rates'] = np.hstack(self.controller_rate).tolist()
            ut.save_pickle(d, self.root_path + '/controller_rates.pkl')

        full_path = full_path.rstrip('/')
        res_dict = ut.load_pickle(full_path+'/reach_from_reach_in_dy_0.00_retry_right=to_goal_1_log.pkl')
        trial = full_path.split('/')[-1]

        if res_dict is None:
            pass
        else: 
            self.controller_rate.append(res_dict['jep_send_rate'])

    def trials_where_mean_stopped_moving(self, full_path):
        if full_path == 'start':
            self.trial_list = []
            return

        if full_path == 'end':
            d = {}
            d['trial_list'] = self.trial_list
            d['num_trials'] = len(self.trial_list)
            ut.save_pickle(d, self.root_path + '/trials_where_ee_stopped_moving.pkl')

        full_path = full_path.rstrip('/')
        log_dict = ut.load_pickle(full_path+'/reach_from_reach_in_dy_0.00_retry_right=to_goal_1_log.pkl')
        res_dict = ut.load_pickle(full_path+'/overall_result.pkl')

        trial = full_path.split('/')[-1]

        if res_dict is None or log_dict['mean_motion_list'] == []:
            pass
        elif log_dict['mean_motion_list'][-1] < 0.001 and res_dict['reached_goal'] != True:
            self.trial_list.append(full_path)


    def count_max_number_of_contacts_per_trial(self, full_path):
        if full_path == 'start':
            self.max_contact_list = []
            return

        if full_path == 'end':
            d = {}
            d['max_contact_list'] = self.max_contact_list
            ut.save_pickle(d, self.root_path + '/max_contact_list.pkl')

        full_path = full_path.rstrip('/')
        res_dict = ut.load_pickle(full_path+'/reach_from_reach_in_dy_0.00_retry_right=to_goal_1_log.pkl')
        trial = full_path.split('/')[-1]

        if res_dict is None:
            pass
        else: 
            self.max_contact_list.append(np.max(res_dict['num_contacts_at_time_instant_list']))

    def compute_success_rate(self, full_path):
        if full_path == 'start':
            self.success_count = 0
            self.fail_count = 0
            self.not_completed_trial_list = []
            self.success_trials_list = []
            return

        if full_path == 'end':
            print self.success_count, 'successful trials'
            print self.fail_count, 'failed trials'
            print self.success_count * 100. / (self.success_count+self.fail_count), '% success rate.'
            print self.success_count + self.fail_count, 'total trials'

            d = {}
            d['successful_trials'] = self.success_trials_list
            d['success_count'] = self.success_count
            d['fail_count'] = self.fail_count
            ut.save_pickle(d, self.root_path + '/combined_results.pkl')
            return

        full_path = full_path.rstrip('/')
        res_dict = ut.load_pickle(full_path+'/overall_result.pkl')
        trial = full_path.split('/')[-1]

        if res_dict is None:
            self.not_completed_trial_list.append(trial)
            self.fail_count += 1
        elif res_dict['reached_goal']:
            self.success_count += 1
            self.success_trials_list.append(trial)
        else:
            self.fail_count += 1

    # what would the success rate for a single reach have been if the
    # stopping force were different.
    def success_rate_for_different_stopping_force(self, full_path):
        stopping_force_list = [5, 10, 15, 20, 25, 30, 40, 50, 100]

        if full_path == 'start':
            self.success_count_list = [0 for f in stopping_force_list]
            self.fail_count_list = [0 for f in stopping_force_list]
            return

        if full_path == 'end':
            d = {}
            for i in range(len(stopping_force_list)):
                print '==== Stopping force :%.1fN ======'%stopping_force_list[i]
                print self.success_count_list[i], 'successful trials'
                print self.fail_count_list[i], 'failed trials'
                print self.success_count_list[i] * 1. / (self.success_count_list[i]+self.fail_count_list[i]), '% success rate.'
                print self.success_count_list[i] + self.fail_count_list[i], 'total trials'
                d[i] = {}
                d[i]['force'] = stopping_force_list[i]
                d[i]['success_count'] = self.success_count_list[i]
                d[i]['fail_count'] = self.fail_count_list[i]
            ut.save_pickle(d, self.root_path + '/success_rate_diff_stopping_force.pkl')
            return

        param_pkl_list = ut.get_bash_command_output('ls -t '+full_path+'/*reach_from*to_goal*param*.pkl')
        param_pkl_list.reverse()
        pkl_list = ut.get_bash_command_output('ls -t '+full_path+'/*reach_from*to_goal*log*.pkl')
        pkl_list.reverse()

        if pkl_list != []:
            pd = ut.load_pickle(param_pkl_list[0])
            stopping_dist_to_goal = pd['stopping_dist_to_goal'] + 0.002

            d = ut.load_pickle(pkl_list[0])
            max_f_l = d['max_force_list']
            max_f_arr = np.array(max_f_l)
            goal = d['local_goal']

            for i,f in enumerate(stopping_force_list):
                idx_arr = np.where(max_f_arr > f)[0]
                if len(idx_arr) == 0:
                    idx = -1
                else:
                    idx = idx_arr[0]

                try:
                    ee = np.matrix(d['ee_pos_list'][idx]).T
                    dist = np.linalg.norm(ee - goal)
                except IndexError:
                    print '-----------------------------------'
                    print full_path
                    print 'idx:', idx
                    print 'len(d[\'ee_pos_list\']):', len(d['ee_pos_list'])
                    print 'len(d[\'max_force_list\']):', len(d['max_force_list'])
                    dist = 200.

                if dist <= stopping_dist_to_goal:
                    self.success_count_list[i] += 1
                else:
                    self.fail_count_list[i] += 1

    def compute_reach_in_force_statistics(self, full_path):
        if full_path == 'start':
            self.single_reach_max_force_list = []
            self.single_reach_95_percentile_force_list = []
            self.single_reach_avg_force_list = []
            self.single_reach_force_count_list_for_avg = []

            self.all_forces_mag_list = []
            self.max_force_list = []
            self.avg_force_list = []
            self.force_count_list_for_avg = []
            return

        if full_path == 'end':
            def compute_overall_avg(avg_list, count_list):
                return np.sum(np.multiply(avg_list, count_list)) / np.sum(count_list)

            d = {}

            d['all_forces_mag_list'] = self.all_forces_mag_list
            d['median_contact_force'] = ss.scoreatpercentile(self.all_forces_mag_list, 50)
            d['first_quartile_contact_force'] = ss.scoreatpercentile(self.all_forces_mag_list, 25)
            d['third_quartile_contact_force'] = ss.scoreatpercentile(self.all_forces_mag_list, 75)
            print ''
            print 'Median contact force:', d['median_contact_force']
            print 'First quartile contact force:', d['first_quartile_contact_force']
            print 'Third quartile contact force', d['third_quartile_contact_force']
            print ''

            avg_max_force = np.mean(self.max_force_list)
            avg_contact_force = compute_overall_avg(self.avg_force_list, self.force_count_list_for_avg)
            print ''
            print 'Avg. max force:', avg_max_force
            print 'Avg. contact force:', avg_contact_force
            print ''

            single_reach_avg_max_force = np.mean(self.single_reach_max_force_list)
            single_reach_avg_contact_force = compute_overall_avg(self.single_reach_avg_force_list,
                                                       self.single_reach_force_count_list_for_avg)
            print ''
            print 'Avg. max force (Single Reach):', single_reach_avg_max_force
            print 'Avg. contact force (Single Reach):', single_reach_avg_contact_force
            print ''
            print ''
            d['avg_max_force'] = avg_max_force
            d['avg_contact_force'] = avg_contact_force
            d['avg_max_force_single_reach'] = single_reach_avg_max_force
            d['avg_contact_force_single_reach'] = single_reach_avg_contact_force
            d['max_force_list'] = self.max_force_list
            d['percentile_95_force_list'] = self.single_reach_95_percentile_force_list
            ut.save_pickle(d, self.root_path + '/reach_in_force_statistics.pkl')
            return

        #pkl_list = ut.get_bash_command_output('ls -t '+full_path+'/*reach_from*to_goal*log*.pkl')
        #pkl_list.reverse()

        # ONLY FOR SINGLE REACH. Advait felt like the ls -t was
        # making nfs real slow.
        pkl = full_path+'/reach_from_reach_in_dy_0.00_retry_right=to_goal_1_log.pkl'
        pkl_list = [pkl]

        for i, pkl in enumerate(pkl_list):
            d = ut.load_pickle(pkl)
            if d == None:
                continue
            f_mag_l = d['all_forces_list']
            if len(f_mag_l) == 0.:
                continue

            if i == 0:
                self.single_reach_max_force_list.append(max(f_mag_l))
                self.single_reach_95_percentile_force_list.append(ss.scoreatpercentile(f_mag_l, 95))
                self.single_reach_avg_force_list.append(np.mean(f_mag_l))
                self.single_reach_force_count_list_for_avg.append(len(f_mag_l))

            self.all_forces_mag_list.extend(f_mag_l)
            self.max_force_list.append(max(f_mag_l))
            self.avg_force_list.append(np.mean(f_mag_l))
            self.force_count_list_for_avg.append(len(f_mag_l))

    def compute_kinematic_statistics(self, full_path):
        if full_path == 'start':
            self.ee_distance = []
            self.execution_time = []
            self.avg_velocity = []
            return

        if full_path == 'end':
            print ''
            print 'Avg. velocity:', np.mean(self.avg_velocity)
            print 'Avg. distance:', np.mean(self.ee_distance)
            print 'Avg. execution time:', np.mean(self.execution_time)
            print ''

            d = {}
            d['avg_velocity_list'] = self.avg_velocity
            d['execution_time_list'] = self.execution_time
            d['ee_distance_list'] = self.ee_distance
            ut.save_pickle(d, self.root_path + '/reach_in_kinematics_statistics.pkl')
            return

        pkl_list = ut.get_bash_command_output('ls -t '+full_path+'/*reach_from*to_goal*log*.pkl')
        pkl_list.reverse()

        for i, pkl in enumerate(pkl_list):
            d = ut.load_pickle(pkl)
            if d == None:
                continue
            
            ee_arr = np.array(d['ee_pos_list'])
            motion_arr = ee_arr[1:] - ee_arr[0:-1]
            ee_dist = np.sum(ut.norm(motion_arr.T))
            
            time_l = d['time_stamp_list']
            exec_time = time_l[-1]-time_l[0]

            self.ee_distance.append(ee_dist)
            self.execution_time.append(exec_time)
            self.avg_velocity.append(ee_dist/exec_time)

    def compute_reaches_for_success(self, full_path):
        if full_path == 'start':
            self.n_reaches_for_success_list = []
            return

        if full_path == 'end':
            d = {}
            d['n_reaches_for_success_list'] = self.n_reaches_for_success_list
            ut.save_pickle(d, self.root_path + '/count_reaches_for_success.pkl')
            return


        full_path = full_path.rstrip('/')
        res_dict = ut.load_pickle(full_path+'/overall_result.pkl')
        trial = full_path.split('/')[-1]

        if res_dict is None:
            self.n_reaches_for_success_list.append(-1)
        elif res_dict['reached_goal']:
            pkl_list = ut.get_bash_command_output('ls -t '+full_path+'/*reach_from*to_goal*log*.pkl')
            if pkl_list != []:
                self.n_reaches_for_success_list.append(len(pkl_list))
        else:
            self.n_reaches_for_success_list.append(-1)


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--dir', action='store', dest='direc',
                 type='string', default = None,
                 help='top level directory with sub-directories for each reach problem.')

    opt, args = p.parse_args()

    if opt.direc == None:
        print 'Specify a root directory.'
        print 'Exiting ...'
        sys.exit()

    crs = ComputeResultStatistics()

    fn_list = []

    fn_list.append(crs.compute_success_rate)
    fn_list.append(crs.compute_kinematic_statistics)
    fn_list.append(crs.compute_reach_in_force_statistics)
    fn_list.append(crs.combine_controller_rates)
    fn_list.append(crs.trials_where_mean_stopped_moving)
    fn_list.append(crs.count_max_number_of_contacts_per_trial)

    # fn_list.append(crs.get_sum_taxel_hist)
    # #fn_list.append(crs.success_rate_for_different_stopping_force)
    # fn_list.append(crs.compute_reaches_for_success)

    crs.traverse_directory(opt.direc, fn_list)





