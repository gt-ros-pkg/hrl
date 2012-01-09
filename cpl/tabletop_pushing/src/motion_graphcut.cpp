// TODO: Figure out includes necessary
// TODO: Make importable

class MotionGraphcut
{
 public:
  MotionGraphcut(double workspace_background_weight = 1.0f,
                 double min_weight = 0.01, double magnitude_thresh=0.1,
                 double flow_gain = 0.3, int arm_grow_radius=2) :
      workspace_background_weight_(workspace_background_weight),
      min_weight_(min_weight), magnitude_thresh_(magnitude_thresh),
      flow_gain_(flow_gain), arm_grow_radius_(arm_grow_radius)
  {
  }

  virtual ~MotionGraphcut()
  {
  }

  /**
   * Segment moving stuff from static stuff using graphcut
   *
   * @param color_frame    The current color image to segment
   * @param depth_frame    The current depth image to segment
   * @param u              Flow dx/dt
   * @param v              Flow dy/dt
   * @param eigen_scores    Scores corresponding to texture at the point
   * @param workspace_mask Binary image where white is valid locations for things
   *                       to be moving at
   * @param arm_locs Image locations of projected arm kinematics
   *
   * @return A binary image where white is the foreground (moving) regions and
   *         black is the background (static) regions
   */
  cv::Mat operator()(cv::Mat& color_frame, cv::Mat& depth_frame,
                     cv::Mat& u, cv::Mat& v, cv::Mat& workspace_mask,
                     cv::Mat& table_heights, ArmModel arm_locs)
  {
    const int R = color_frame.rows;
    const int C = color_frame.cols;
    int num_nodes = R*C;
    int num_edges = ((C-1)*3+1)*(R-1)+(C-1);
    GraphType *g;
    g = new GraphType(num_nodes, num_edges);

#ifdef VISUALIZE_GRAPH_WEIGHTS
    cv::Mat fg_weights(color_frame.size(), CV_32FC1);
    cv::Mat bg_weights(color_frame.size(), CV_32FC1);
    cv::Mat table_weights(color_frame.size(), CV_32FC1);
#endif // VISUALIZE_GRAPH_WEIGHTS
#ifdef VISUALIZE_GRAPH_EDGE_WEIGHTS
    cv::Mat left_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat up_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat up_left_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
#endif // VISUALIZE_GRAPH_EDGE_WEIGHTS

    for (int r = 0; r < R; ++r)
    {
      for (int c = 0; c < C; ++c)
      {
        g->add_node();
        float magnitude = std::sqrt(u.at<float>(r,c)*u.at<float>(r,c) +
                                    v.at<float>(r,c)*v.at<float>(r,c));
        // Check if we are hardcoding this spot to background
        if (workspace_mask.at<uchar>(r,c) == 0)
        {
          g->add_tweights(r*C+c, min_weight_, workspace_background_weight_);

#ifdef VISUALIZE_GRAPH_WEIGHTS
          fg_weights.at<float>(r,c) = min_weight_;
          bg_weights.at<float>(r,c) = workspace_background_weight_;
          table_weights.at<float>(r,c) = min_weight_;
#endif // VISUALIZE_GRAPH_WEIGHTS
          continue;
        }
        const float mag_score = max(getFlowFGScore(magnitude), min_weight_);
        const float table_score = max(getTableScore(color_frame.at<cv::Vec3f>(r,c),
            fabs(table_heights.at<float>(r,c))), min_weight_);
        const float not_mag_score = max(1.0 - mag_score, min_weight_);
        const float bg_score = not_mag_score + table_score;
        g->add_tweights(r*C+c, mag_score, bg_score);
#ifdef VISUALIZE_GRAPH_WEIGHTS
        fg_weights.at<float>(r,c) = mag_score;
        bg_weights.at<float>(r,c) = bg_score;
        table_weights.at<float>(r,c) = table_score;
#endif // VISUALIZE_GRAPH_WEIGHTS
      }
    }
    for (int r = 0; r < R; ++r)
    {
      for (int c = 0; c < C; ++c)
      {
        // Connect node to previous ones
        if (c > 0)
        {
          // Add left-link
          float w_l = getEdgeWeight(color_frame.at<cv::Vec3f>(r,c),
                                    depth_frame.at<float>(r,c),
                                    color_frame.at<cv::Vec3f>(r,c-1),
                                    depth_frame.at<float>(r,c-1));
          g->add_edge(r*C+c, r*C+c-1, /*capacities*/ w_l, w_l);
#ifdef VISUALIZE_GRAPH_EDGE_WEIGHTS
          left_weights.at<float>(r,c) = w_l;
#endif // VISUALIZE_EDGE_GRAPH_WEIGHTS
        }
        if (r > 0)
        {
          // Add up-link
          float w_u = getEdgeWeight(color_frame.at<cv::Vec3f>(r,c),
                                    depth_frame.at<float>(r,c),
                                    color_frame.at<cv::Vec3f>(r-1,c),
                                    depth_frame.at<float>(r-1,c));
          g->add_edge(r*C+c, (r-1)*C+c, /*capacities*/ w_u, w_u);
#ifdef VISUALIZE_GRAPH_EDGE_WEIGHTS
          up_weights.at<float>(r,c) = w_u;
#endif // VISUALIZE_GRAPH_EDGE_WEIGHTS
          // Add up-left-link
          if (c > 0)
          {
            float w_ul = getEdgeWeight(color_frame.at<cv::Vec3f>(r,c),
                                       depth_frame.at<float>(r,c),
                                       color_frame.at<cv::Vec3f>(r-1,c-1),
                                       depth_frame.at<float>(r-1,c-1));
            g->add_edge(r*C+c, (r-1)*C+c-1, /*capacities*/ w_ul, w_ul);
#ifdef VISUALIZE_GRAPH_EDGE_WEIGHTS
          up_left_weights.at<float>(r,c) = w_ul;
#endif // VISUALIZE_GRAPH_EDGE_WEIGHTS
          }
        }
      }
    }

#ifdef VISUALIZE_GRAPH_WEIGHTS
    cv::imshow("fg_weights", fg_weights);
    cv::imshow("bg_weights", bg_weights);
    // double table_max=0;
    // cv::minMaxLoc(table_weights, NULL, &table_max);
    // table_weights /= table_max;
    cv::imshow("table_weights", table_weights);
#endif // VISUALIZE_GRAPH_WEIGHTS
#ifdef VISUALIZE_GRAPH_EDGE_WEIGHTS
    double up_max = 1.0;
    cv::minMaxLoc(up_weights, NULL, &up_max);
    up_weights /= up_max;

    cv::imshow("up_weights", up_weights);
    double left_max = 1.0;
    cv::minMaxLoc(left_weights, NULL, &left_max);
    left_weights /= left_max;
    cv::imshow("left_weights", left_weights);
    double up_left_max = 1.0;
    cv::minMaxLoc(up_left_weights, NULL, &up_left_max);
    up_left_weights /= up_max;
    cv::imshow("up_left_weights", up_left_weights);
#endif // VISUALIZE_GRAPH_EDGE_WEIGHTS

    // int flow = g->maxflow(false);
    g->maxflow(false);

    // Convert output into image
    cv::Mat segs = convertFlowResultsToCvMat(g, R, C);
    delete g;
    return segs;
  }

  /**
   * Method to segment the arm from the rest of the stuff moving in the scene
   *
   * @param color_frame    Current color frame
   * @param depth_frame    Current depth frame
   * @param moving_mask    Binary image representing where the moving stuff is
   * @param workspace_mask Binary image where white is valid locations for things
   *                       to be moving at
   * @param arms           Position of the arm projected into the image frame
   *
   * @return               Mask of the predicted arm in the image
   */
  cv::Mat segmentRobotArm(cv::Mat& color_frame_in, cv::Mat& depth_frame_in,
                          cv::Mat& workspace_mask_in, ArmModel& arms,
                          int min_arm_x, int max_arm_x, int min_arm_y,
                          int max_arm_y)
  {
    // NOTE: We examine only a subwindow in the image to avoid too make things
    // more efficient
    const int crop_min_x = max(0, min_arm_x - arm_search_radius_);
    const int crop_max_x = min(color_frame_in.cols,
                                    max_arm_x + arm_search_radius_);
    const int crop_min_y = max(0, min_arm_y - arm_search_radius_);
    const int crop_max_y = min(color_frame_in.rows,
                                    max_arm_y + arm_search_radius_);
    cv::Rect roi(crop_min_x, crop_min_y, crop_max_x-crop_min_x,
                 crop_max_y-crop_min_y);
    cv::Mat color_frame = color_frame_in(roi);
    cv::Mat depth_frame = depth_frame_in(roi);
    cv::Mat workspace_mask = workspace_mask_in(roi);

    const int R = color_frame.rows;
    const int C = color_frame.cols;

    int num_nodes = R*C;
    int num_edges = ((C-1)*3+1)*(R-1)+(C-1);
    GraphType *g;
    g = new GraphType(num_nodes, num_edges);

#ifdef VISUALIZE_ARM_GRAPH_WEIGHTS
    cv::Mat fg_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat bg_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat dist_img(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
#endif // VISUALIZE_GRAPH_WEIGHTS
#ifdef VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS
    cv::Mat left_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat up_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat up_left_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
#endif // VISUALIZE_ARM_GRAPH_WEIGHTS

    // One gaussian estimated from wrist to elbow, elbow to forearm and a
    // separate one is estimated from the gripper tip to wrist
    if (arms[0].size() == 0)
    {
      ROS_WARN_STREAM("No hands!");
    }
    if (arms[1].size() == 0)
    {
      ROS_WARN_STREAM("No arms!");
    }
    std::vector<cv::Vec3f> hand_stats = getImagePointGaussian(color_frame,
                                                              arms[0],
                                                              crop_min_x,
                                                              crop_min_y);
    std::vector<cv::Vec3f> arm_stats = getImagePointGaussian(color_frame,
                                                             arms[1],
                                                             crop_min_x,
                                                             crop_min_y);
    // Tie weights to fg / bg
    for (int r = 0; r < R; ++r)
    {
      for (int c = 0; c < C; ++c)
      {
        g->add_node();
#ifdef USE_WORKSPACE_MASK_FOR_ARM
        if (workspace_mask.at<uchar>(r,c) == 0)
        {
          g->add_tweights(r*C+c, min_weight_, workspace_background_weight_);
#ifdef VISUALIZE_ARM_GRAPH_WEIGHTS
          fg_weights.at<float>(r,c) = min_weight_;
          bg_weights.at<float>(r,c) = workspace_background_weight_;
          dist_img.at<float>(r,c) = min_weight_;
#endif // VISUALIZE_GRAPH_WEIGHTS
          continue;
        }
#endif // USE_WORKSPACE_MASK_FOR_ARM
#ifdef VISUALIZE_ARM_GRAPH_WEIGHTS
        const float me_score = max(getArmFGScore(color_frame, depth_frame, r, c,
                                                 arm_stats, hand_stats, arms,
                                                 roi, dist_img), min_weight_);
#else // VISUALIZE_ARM_GRAPH_WEIGHTS
        const float me_score = max(getArmFGScore(color_frame, depth_frame, r, c,
                                                 arm_stats, hand_stats, arms,
                                                 roi), min_weight_);
#endif // VISUALIZE_ARM_GRAPH_WEIGHTS
        const float not_me_score = max(1.0 - me_score, min_weight_);
        g->add_tweights(r*C+c, me_score, not_me_score);
#ifdef VISUALIZE_ARM_GRAPH_WEIGHTS
        fg_weights.at<float>(r,c) = me_score;
        bg_weights.at<float>(r,c) = not_me_score;
#endif // VISUALIZE_ARM_GRAPH_WEIGHTS
      }
    }
    // Add edge weights
    for (int r = 0; r < R; ++r)
    {
      for (int c = 0; c < C; ++c)
      {
        // Connect node to previous ones
        if (c > 0)
        {
          // Add left-link
          float w_l = getEdgeWeight(color_frame.at<cv::Vec3f>(r,c),
                                    depth_frame.at<float>(r,c),
                                    color_frame.at<cv::Vec3f>(r,c-1),
                                    depth_frame.at<float>(r,c-1));
          g->add_edge(r*C+c, r*C+c-1, /*capacities*/ w_l, w_l);
#ifdef VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS
          left_weights.at<float>(r,c) = w_l;
#endif // VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS
        }
        if (r > 0)
        {
          // Add up-link
          float w_u = getEdgeWeight(color_frame.at<cv::Vec3f>(r,c),
                                    depth_frame.at<float>(r,c),
                                    color_frame.at<cv::Vec3f>(r-1,c),
                                    depth_frame.at<float>(r-1,c));
          g->add_edge(r*C+c, (r-1)*C+c, /*capacities*/ w_u, w_u);
#ifdef VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS
          up_weights.at<float>(r,c) = w_u;
#endif // VISUALIZE_ARM_GRAPH_WEIGHTS

          // Add up-left-link
          if (c > 0)
          {
            float w_ul = getEdgeWeight(color_frame.at<cv::Vec3f>(r,c),
                                       depth_frame.at<float>(r,c),
                                       color_frame.at<cv::Vec3f>(r-1,c-1),
                                       depth_frame.at<float>(r-1,c-1));
            g->add_edge(r*C+c, (r-1)*C+c-1, /*capacities*/ w_ul, w_ul);
#ifdef VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS
            up_left_weights.at<float>(r,c) = w_ul;
#endif // VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS

          }
        }
      }
    }
#ifdef VISUALIZE_ARM_GRAPH_WEIGHTS
    cv::imshow("fg_weights_arm", fg_weights);
    cv::imshow("bg_weights_arm", bg_weights);
    cv::imshow("arm_dist_scores", dist_img);
#endif // VISUALIZE_ARM_GRAPH_WEIGHTS
#ifdef VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS
    double up_max = 1.0;
    cv::minMaxLoc(up_weights, NULL, &up_max);
    up_weights /= up_max;
    double left_max = 1.0;
    cv::minMaxLoc(left_weights, NULL, &left_max);
    left_weights /= left_max;
    double up_left_max = 1.0;
    cv::minMaxLoc(up_left_weights, NULL, &up_left_max);
    up_left_weights /= up_max;
    cv::imshow("up_weights_arm", up_weights);
    cv::imshow("left_weights_arm", left_weights);
    cv::imshow("up_left_weights_arm", up_left_weights);
#endif // VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS

    // int flow = g->maxflow(false);
    g->maxflow(false);

    // Convert output into image
    cv::Mat segs = convertFlowResultsToCvMat(g, R, C, roi,
                                             color_frame_in.size());
    delete g;
    return segs;
  }

  float getFlowFGScore(float magnitude)
  {
    return min(1.0, max(flow_gain_*exp(magnitude), 0.0));
  }

  float getTableScore(cv::Vec3f cur_c, float height_from_table)
  {
    const float dist_score = exp(-height_from_table/table_height_var_);
#ifndef USE_TABLE_COLOR_ESTIMATE
    return min(1.0, max(dist_score, 0.0));
#else // USE_TABLE_COLOR_ESTIMATE
    const float h_score = 1.0-fabs(cur_c[0] - table_stats_[0][0])/(
        table_stats_[1][0] + arm_color_var_add_);
    const float s_score = 1.0-fabs(cur_c[1] - table_stats_[0][1])/(
        table_stats_[1][1] + arm_color_var_add_);
    const float table_score = (h_score + s_score)/2.0+0.5*dist_score;
    return table_score;
#endif // USE_TABLE_COLOR_ESTIMATE
  }
#ifdef VISUALIZE_ARM_GRAPH_WEIGHTS
  float getArmFGScore(cv::Mat& color_frame, cv::Mat& depth_frame, int r, int c,
                      std::vector<cv::Vec3f>& arm_stats,
                      std::vector<cv::Vec3f>& hand_stats, ArmModel& arms,
                      cv::Rect& roi, cv::Mat& dist_img)
#else // VISUALIZE_ARM_GRAPH_WEIGHTS
  float getArmFGScore(cv::Mat& color_frame, cv::Mat& depth_frame, int r, int c,
                      std::vector<cv::Vec3f>& arm_stats,
                      std::vector<cv::Vec3f>& hand_stats, ArmModel& arms,
                      cv::Rect& roi)
#endif // VISUALIZE_ARM_GRAPH_WEIGHTS
  {
    const float dist_score = exp(-arms.distanceToArm(
        cv::Point2f(c+roi.x,r+roi.y), depth_frame)/arm_dist_var_);
#ifdef VISUALIZE_ARM_GRAPH_WEIGHTS
    dist_img.at<float>(r,c) = dist_score;
#endif // VISUALIZE_ARM_GRAPH_WEIGHTS
    cv::Vec3f cur_c = color_frame.at<cv::Vec3f>(r,c);
    const float arm_h_score = 1.0-fabs(cur_c[0] - arm_stats[0][0])/(
        arm_stats[1][0] + arm_color_var_add_);
    const float arm_s_score = 1.0-fabs(cur_c[1] - arm_stats[0][1])/(
        arm_stats[1][1] + arm_color_var_add_);
    const float arm_v_score = 1.0-fabs(cur_c[2] - arm_stats[0][2])/(
        arm_stats[1][2] + arm_color_var_add_);
    const float arm_score = (arm_alpha_*(arm_h_score + arm_s_score +
                                         arm_v_score)/3.0 +
                             arm_beta_*dist_score);
    const float hand_h_score = 1.0-fabs(cur_c[0] - hand_stats[0][0])/(
        hand_stats[1][0] + arm_color_var_add_);
    const float hand_s_score = 1.0-fabs(cur_c[1] - hand_stats[0][1])/(
        hand_stats[1][1] + arm_color_var_add_);
    const float hand_v_score = 1.0-fabs(cur_c[2] - hand_stats[0][2])/(
        hand_stats[1][2] + arm_color_var_add_);
    const float hand_score = (arm_alpha_*(hand_h_score + hand_s_score +
                                          hand_v_score) / 3.0 +
                              arm_beta_*dist_score);
    return max(hand_score, arm_score);
  }

  std::vector<cv::Vec3f> getImagePointGaussian(cv::Mat& color_frame,
                                               std::vector<cv::Point> points,
                                               int min_x=0, int min_y=0)
  {
    // Calculate color means and variances
    const int C = color_frame.cols;
    const int R = color_frame.rows;
    int pixel_count = 0;
    cv::Vec3f means;
    means[0] = 0.0;
    means[1] = 0.0;
    means[2] = 0.0;
    for (unsigned int i = 0; i < points.size(); ++i)
    {
      for (int r = max(0, points[i].y - min_y - arm_grow_radius_);
           r < min(points[i].y - min_y + arm_grow_radius_, R); ++r)
      {
        for (int c = max(0, points[i].x - min_x - arm_grow_radius_);
             c < min(points[i].x - min_x + arm_grow_radius_, C); ++c)
        {
          cv::Vec3f cur_color = color_frame.at<cv::Vec3f>(r,c);
          means += cur_color;
          ++pixel_count;
        }
      }
    }
    if (pixel_count > 0)
    {
      means[0] /= pixel_count;
      means[1] /= pixel_count;
      means[2] /= pixel_count;
    }
    else
    {
      ROS_WARN_STREAM("Calculating stats for 0 pixels");
    }
    cv::Vec3f vars;
    vars[0] = 0.0;
    vars[1] = 0.0;
    vars[2] = 0.0;
    for (unsigned int i = 0; i < points.size(); ++i)
    {
      for (int r = max(0, points[i].y - min_y -arm_grow_radius_);
           r < min(points[i].y - min_y + arm_grow_radius_, R); ++r)
      {
        for (int c = max(0, points[i].x - min_x - arm_grow_radius_);
             c < min(points[i].x - min_x + arm_grow_radius_, C); ++c)
        {
          cv::Vec3f diff = color_frame.at<cv::Vec3f>(r,c);
          diff = diff.mul(diff);
          vars += diff;
        }
      }
    }
    vars[0] /=  (pixel_count+1.0);
    vars[1] /=  (pixel_count+1.0);
    vars[2] /=  (pixel_count+1.0);
    std::vector<cv::Vec3f> stats;
    stats.push_back(means);
    stats.push_back(vars);
    return stats;
  }

  void setTableColorStats(cv::Mat& color_frame, std::vector<cv::Point>& pts)
  {
    cv::Mat color_frame_hsv(color_frame.size(), color_frame.type());
    cv::cvtColor(color_frame, color_frame_hsv, CV_BGR2HSV);
    cv::Mat color_frame_f(color_frame_hsv.size(), CV_32FC3);
    color_frame_hsv.convertTo(color_frame_f, CV_32FC3, 1.0/255, 0);
    table_stats_ = getImagePointGaussian(color_frame_f, pts);
  }

  cv::Mat convertFlowResultsToCvMat(GraphType *g, int R, int C)
  {
    cv::Mat segs(R, C, CV_8UC1, cv::Scalar(0));
    for (int r = 0; r < R; ++r)
    {
      uchar* seg_row = segs.ptr<uchar>(r);
      for (int c = 0; c < C; ++c)
      {
        int label = (g->what_segment(r*C+c) == GraphType::SOURCE);
        seg_row[c] = label*255;
      }
    }
    return segs;
  }

  cv::Mat convertFlowResultsToCvMat(GraphType *g, int R, int C,
                                    cv::Rect roi, cv::Size out_size)
  {
    cv::Mat segs(out_size, CV_8UC1, cv::Scalar(0));
    for (int r = 0; r < R; ++r)
    {
      for (int c = 0; c < C; ++c)
      {
        int label = (g->what_segment(r*C+c) == GraphType::SOURCE);
        segs.at<uchar>(r+roi.y, c+roi.x) = label*255;
      }
    }
    return segs;
  }

  float getEdgeWeight(cv::Vec3f c0, float d0, cv::Vec3f c1, float d1)
  {
    cv::Vec3f c_d = c0-c1;
    float w_d = d0-d1;
    float w_c = w_c_alpha_*exp(fabs(c_d[0])) + w_c_beta_*exp(fabs(c_d[1])) +
        /*w_c_beta_*exp(fabs(c_d[2])) +*/ w_c_gamma_*exp(fabs(w_d));
    return w_c;
  }

  enum SegLabels
  {
    MOVING,
    ARM,
    TABLE,
    BG
  };

  double workspace_background_weight_;
  double min_weight_;
  double w_c_alpha_;
  double w_c_beta_;
  double w_c_gamma_;
  double magnitude_thresh_;
  double flow_gain_;
  double table_height_var_;
  double arm_dist_var_;
  double arm_color_var_add_;
  double arm_alpha_;
  double arm_beta_;
  int arm_grow_radius_;
  int arm_search_radius_;
  std::vector<cv::Vec3f> table_stats_;
};
