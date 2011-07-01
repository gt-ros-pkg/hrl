#! /usr/bin/python

from hrl_pr2_arms.pid_controller import PIDController

def main():
    arm = sys.argv[1]
    mode = sys.argv[2]
    assert arm in ['r', 'l']
    assert mode in ['jt1']

    if mode == 'jt1':
        pr2_jt_arm = PR2ArmJointTrajectory('r')
        pr2_jt_arm.set_ep([0.]*7, 15)

    if mode == 'jt2':
        pass

if __name__ == "__main__":
    main()
