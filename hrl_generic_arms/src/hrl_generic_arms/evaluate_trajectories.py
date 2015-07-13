
from hrl_generic_arms.ep_control import EPGenerator, EPC, EPStopConditions

##
# Runs a series of trials by generating new EPGenerators on a list of different
# parameters.
# @param ep_gen_eval_creator Function which takes a parameter object and returns an EPGenerator
# @param ep_gen_eval_params a list of trials to be evaluated
# @param time_step: Time between successive calls to equi_pt_generator
# @param timeout - time after which the epc motion will stop.
# @return (dict, list) A dictionary of the result statistics and a list of the results.
def evaluate_trajectories(ep_gen_setup_creator, ep_gen_setup_params,
                          ep_gen_eval_creator, ep_gen_eval_params, time_step, timeout):
    epc = EPC('eval_trajs')
    result_dict = {'test not run' : len(ep_gen_eval_params)}
    result_list = ['test not run'] * len(ep_gen_eval_params)
    i = 0
    for ep_gen_setup_param, ep_gen_eval_param in zip(ep_gen_setup_params, ep_gen_eval_params):
        ep_gen_setup = ep_gen_eval_creator(ep_gen_param)
        stop = epc.epc_motion(ep_gen_setup, time_step, timeout)
        if stop == EPStopConditions.ROSPY_SHUTDOWN
            break
        if stop == EPStopConditions.SUCCESSFUL:
            ep_gen_eval = ep_gen_eval_creator(ep_gen_eval_param)
            stop = epc.epc_motion(ep_gen_eval, time_step, timeout)
            if stop == EPStopConditions.ROSPY_SHUTDOWN
                break
        else:
            rospy.loginfo('[evaluate_trajectories] Unsuccessful setup, not doing trajectory.')
            stop = 'unsuccessful setup'
        if stop not in result_dict:
            result_dict[stop] = 1
        else:
            result_dict[stop] += 1
        result_dict['test not run'] -= 1
        result_list[i] = stop
        i += 1
        yield result_dict, result_list

