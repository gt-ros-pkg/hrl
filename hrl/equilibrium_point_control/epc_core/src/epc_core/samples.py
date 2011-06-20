


    ## Pull back along a straight line (-ve x direction)
    # @param arm - 'right_arm' or 'left_arm'
    # @param distance - how far back to pull.
    def pull_back_cartesian_control(self, arm, distance, logging_fn):
        cep, _ = self.robot.get_cep_jtt(arm)
        cep_end = cep + distance * np.matrix([-1., 0., 0.]).T
        self.dist_left = distance

        def eq_gen_pull_back(cep):
            logging_fn(arm)
            if self.dist_left <= 0.:
                return 'done', None
            step_size = 0.01
            cep[0,0] -= step_size
            self.dist_left -= step_size
            if cep[0,0] < 0.4:
                return 'very close to the body: %.3f'%cep[0,0], None
            return '', (cep, None)
        
        arg_list = [cep]
        s = self.epc_motion(eq_gen_pull_back, 0.1, arm, arg_list,
                    control_function = self.robot.set_cep_jtt)
        print s

    def move_till_hit(self, arm, vec=np.matrix([0.3,0.,0.]).T, force_threshold=3.0,
                      speed=0.1, bias_FT=True):
        unit_vec =  vec/np.linalg.norm(vec)
        time_step = 0.1
        dist = np.linalg.norm(vec)
        step_size = speed * time_step
        cep_start, _ = self.robot.get_cep_jtt(arm)
        cep = copy.copy(cep_start)
        def eq_gen(cep):
            force = self.robot.get_wrist_force(arm, base_frame = True)
            force_projection = force.T*unit_vec *-1 # projection in direction opposite to motion.
            print 'force_projection:', force_projection
            if force_projection>force_threshold:
                return 'done', None
            elif np.linalg.norm(force)>45.:
                return 'large force', None
            elif np.linalg.norm(cep_start-cep) >= dist:
                return 'reached without contact', None
            else:
                cep_t = cep + unit_vec * step_size
                cep[0,0] = cep_t[0,0]
                cep[1,0] = cep_t[1,0]
                cep[2,0] = cep_t[2,0]
                return '', (cep, None)

        if bias_FT:
            self.robot.bias_wrist_ft(arm)
        rospy.sleep(0.5)
        return self.epc_motion(eq_gen, time_step, arm, [cep],
                               control_function = self.robot.set_cep_jtt)

    def cep_gen_surface_follow(self, arm, move_dir, force_threshold,
                               cep, cep_start):
        wrist_force = self.robot.get_wrist_force(arm, base_frame=True)
        if wrist_force[0,0] < -3.:
            cep[0,0] -= 0.002
        if wrist_force[0,0] > -1.:
            cep[0,0] += 0.003
    
        if cep[0,0] > (cep_start[0,0]+0.05):
            cep[0,0] = cep_start[0,0]+0.05
    
        step_size = 0.002
        cep_t = cep + move_dir * step_size
        cep[0,0] = cep_t[0,0]
        cep[1,0] = cep_t[1,0]
        cep[2,0] = cep_t[2,0]

        v = cep - cep_start
        if (wrist_force.T * move_dir)[0,0] < -force_threshold:
            stop = 'got a hook'
        elif np.linalg.norm(wrist_force) > 50.:
            stop = 'force is large %f'%(np.linalg.norm(wrist_force))
        elif (v.T * move_dir)[0,0] > 0.20:
            stop = 'moved a lot without a hook'
        else:
            stop = ''
        return stop, (cep, None)


    def go_pose(self, arm, goal_pos, goal_rot):
        time_step = 0.02
        def eq_gen():
            jep = self.robot.get_jep(arm)
            cep, rot_cep = self.robot.arms.FK_all(arm, jep)
            q = self.robot.get_joint_angles(arm)
            ee, ee_rot = self.robot.arms.FK_all(arm, q)
            err = goal_pos - ee
            err_mag = np.linalg.norm(err)
            step_size = min(0.001, err_mag)
            cep_new = cep + err/err_mag * step_size
            jep_new = self.robot.IK(arm, cep_new, rot_cep, jep)

            stop = ''
            if len(jep_new) != 7 or jep_new is None:
                stop = "IK failure"
                return stop, None
            if err_mag < 0.01:
                stop = 'Reached'
                return stop, None

            return stop, (jep_new, time_step*1.2)
        
        return self.epc_motion(eq_gen, time_step, arm, [],
                               control_function=self.robot.set_jep)

    def rot_mat_interpolate(self, start_rot, end_rot, num):
        start_quat = tr.matrix_to_quaternion(start_rot)
        end_quat = tr.matrix_to_quaternion(end_rot)
        mat_list = []
        for fraction in np.linspace(0, 1, num):
            cur_quat = tf_trans.quaternion_slerp(start_quat, end_quat, fraction)
            mat_list.append(tr.quaternion_to_matrix(cur_quat))
        return mat_list

# mag = np.sqrt(np.dot(quat_a, quat_a) * np.dot(quat_b, quat_b))
# assert mag != 0
# return np.arccos(np.dot(quat_a, quat_b) / mag)
        

    def interpolated_linear_trajectory(self, arm, start_pos, start_rot, end_pos, end_rot, 
                                                  offset_pos, offset_rot):
        time_step = 0.02
        velocity = 0.01
        dist = np.linalg.norm(np.array(end_pos) - np.array(start_pos))
        time_req = dist / velocity
        steps_req = time_req / time_step
        self.pos_waypoints = np.dstack([np.linspace(start_pos[0], end_pos[0], steps_req), 
                                        np.linspace(start_pos[1], end_pos[1], steps_req), 
                                        np.linspace(start_pos[2], end_pos[2], steps_req)])[0]
        self.rot_waypoints = self.rot_mat_interpolate(start_rot, end_rot, steps_req)
        self.prev_err_mag = 0
        self.eq_gen_step = 0

        def eq_gen():
            jep = self.robot.get_jep(arm)
            cep, rot_cep = self.robot.arms.FK_all(arm, jep)
            q = self.robot.get_joint_angles(arm)
            ee_pos, ee_rot = self.robot.arms.FK_all(arm, q)
            cur_goal_pos = self.pos_waypoints[self.eq_gen_step]
            cur_goal_rot = self.rot_waypoints[self.eq_gen_step]

            # calculate position error
            err_pos = cur_goal_pos - ee_pos
            
            # calculate rotation error
            err_rot =  ee_rot.T * cur_goal_rot
            angle, direc, point = tf_trans.rotation_from_matrix(err_rot)
            cep_rot_new = tf_trans.rotation_matrix(x * angle, direc, point)
            ang_step_size = min(0.01, err_angle)

            cep_new = cep + err/err_pos_mag * pos_step_size
            jep_new = self.robot.IK(arm, cep_new, rot_cep, jep)

            self.eq_gen_step += 1

            stop = ''
            if len(jep_new) != 7 or jep_new is None:
                stop = "IK failure"
                return stop, None
            if err_mag < 0.01:
                stop = 'Reached'
                return stop, None

            return stop, (jep_new, time_step*1.2)
        
        return self.epc_motion(eq_gen, time_step, arm, [],
                               control_function=self.robot.set_jep)


    def go_jep(self, arm , goal_jep, speed=math.radians(20),
               rapid_call_func = None):
        start_jep = self.robot.get_jep(arm)
        diff_jep = np.array(goal_jep) - np.array(start_jep)
        time_step = 0.02
        max_ch = np.max(np.abs(diff_jep))
        total_time = max_ch / speed
        n_steps = max(np.round(total_time / time_step + 0.5), 1)
        jep_step = diff_jep / n_steps
        step_num = 0
        jep = copy.copy(start_jep)

        def eq_gen(l):
            jep = l[0]
            step_num = l[1]
            if step_num < n_steps:
                q = list(np.array(jep) + jep_step)
                stop = ''
            else:
                q = None
                stop = 'Reached'
            step_num += 1
            l[0] = q
            l[1] = step_num
            return stop, (q, time_step*1.2)
        
        return self.epc_motion(eq_gen, time_step, arm, [[jep, step_num]],
                               control_function=self.robot.set_jep,
                               rapid_call_func = rapid_call_func)



if __name__ == '__main__':
    import pr2_arms.pr2_arms as pa
    rospy.init_node('epc_pr2', anonymous = True)
    rospy.logout('epc_pr2: ready')

    pr2_arms = pa.PR2Arms()
    epc = EPC(pr2_arms)

    r_arm, l_arm = 0, 1
    arm = r_arm

    if False:
        q = epc.robot.get_joint_angles(arm)
        epc.robot.set_jep(arm, q)
        ea = [0, 0, 0, 0, 0, 0, 0]
        raw_input('Hit ENTER to go_jep')
        epc.go_jep(arm, ea, math.radians(30.))

    if False:
        raw_input('Hit ENTER to go_jep')
        q = epc.robot.get_joint_angles(arm)
        epc.robot.set_jep(arm, pr2_arms.wrap_angles(q))
        p, r = epc.robot.arms.FK_all(arm, q)
        q = epc.robot.IK(arm, p, r)
        epc.go_jep(arm, q, math.radians(30.))
        
        raw_input('Hit ENTER to go_pose')
        goal_rot = r
        goal_pos = p + np.matrix([0.2, 0., 0.]).T
#goal_pos = np.mat([0.889, -0.1348, -0.0868]).T
        res = epc.go_pose(arm, goal_pos, goal_rot)
        print 'go_pose result:', res[0]
        print 'goal_pos:', goal_pos.A1

        q = epc.robot.get_joint_angles(arm)
        p, r = epc.robot.arms.FK_all(arm, q)
        print 'current position:', p.A1

    if True:
        start_pos = [0, 0, 0]
        end_pos = [0.05, 0.02, -0.03]
        start_rot, end_rot = tr.rotX(1), tr.rotY(1)
        epc.interpolated_linear_trajectory(arm, start_pos, start_rot, end_pos, end_rot)





