

class T(object):

    def linear_trajectory(self, start_pose, end_pose, tool_frame, velocity, rate=20):
        """Commands arm to follow a linear trajectory, controlling the tool's frame
           and using PD equilibrium point control to keep the tool fixed on this
           trajectory
           @param start_pose PoseStamped containing start position
           @param end_pose PoseStamped containing end position
           @param tool_frame string indicating tool's frame
        """
        # form desired trajectory
        for t_B_ei in trajectory:
            # find current tool location
            t_B_c = 
            # find error in position
            # find error in angle
            # find control input
            # find commanded frame of tool
            # transform commanded frame into wrist
            # find IK solution
            # command joints

