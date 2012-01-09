class FlowClustering
{
 public:
  FlowClustering(FeatureTracker* ft,
                    int kmeans_max_iter=200, double kmeans_epsilon=0.5,
                    int kmeans_tries=5, int affine_estimate_radius=5,
                    double surf_hessian=100) :
      ft_(ft),
      kmeans_max_iter_(kmeans_max_iter), kmeans_epsilon_(kmeans_epsilon),
      kmeans_tries_(kmeans_tries),
      affine_estimate_radius_(affine_estimate_radius)
  {
    // Create derivative kernels for flow calculation
    cv::getDerivKernels(dy_kernel_, dx_kernel_, 1, 0, CV_SCHARR, true, CV_32F);
    cv::flip(dy_kernel_, dy_kernel_, -1);
    cv::transpose(dy_kernel_, dx_kernel_);
  }

  AffineFlowMeasures clusterFlowFields(cv::Mat& color_img, cv::Mat& depth_img,
                                       cv::Mat& u, cv::Mat& v, cv::Mat& mask)
  {
    // return clusterFlowFieldsKMeans(color_img, depth_img, u, v, mask);
    return clusterSparseFlowKMeans(color_img, depth_img, u, v, mask);
  }

  AffineFlowMeasures clusterFlowFieldsKMeans(cv::Mat& color_img,
                                             cv::Mat& depth_img,
                                             cv::Mat& u, cv::Mat& v,
                                             cv::Mat& mask)
  {
    // Setup the samples as the flow vectors for the segmented moving region
    AffineFlowMeasures points;
    points.clear();
    for (int r = 0; r < mask.rows; ++r)
    {
      uchar* mask_row = mask.ptr<uchar>(r);
      for (int c = 0; c < mask.cols; ++c)
      {
        if ( mask_row[c] > 0)
        {
          AffineFlowMeasure p;
          p.u = u.at<float>(r,c);
          p.v = v.at<float>(r,c);
          p.x = c;
          p.y = r;
          p.a = estimateAffineTransform(u, v, r, c, affine_estimate_radius_);
          if (std::sqrt(p.u*p.u+p.v*p.v) > 1.0 )
            points.push_back(p);
        }
      }
    }

    if (points.size() < 1)
    {
      AffineFlowMeasures cluster_centers;
      cluster_centers.clear();
      return cluster_centers;
    }
    AffineFlowMeasures cluster_centers = clusterAffineKMeans(color_img, u, v,
                                                             points);
    return cluster_centers;
  }

  AffineFlowMeasures clusterFlowFieldsRANSAC(cv::Mat& color_img,
                                             cv::Mat& depth_img,
                                             cv::Mat& u, cv::Mat& v,
                                             cv::Mat& mask)
  {
    AffineFlowMeasures points;
    for (int r = 0; r < mask.rows; ++r)
    {
      uchar* mask_row = mask.ptr<uchar>(r);
      for (int c = 0; c < mask.cols; ++c)
      {
        if ( mask_row[c] > 0)
        {
          AffineFlowMeasure p;
          p.u = u.at<float>(r,c);
          p.v = v.at<float>(r,c);
          p.x = c;
          p.y = r;
          if (std::sqrt(p.u*p.u+p.v*p.v) > 1.0 )
            points.push_back(p);
        }
      }
    }

    AffineFlowMeasures cluster_centers;
    if (points.size() < 1)
    {
      cluster_centers.clear();
      return cluster_centers;
    }

    // Perform RANSAC itteratively on the affine estimates to cluster a set of
    // affine movement regions
    int k = 0;
    AffineFlowMeasures active_points = points;
    while (active_points.size() > min_affine_point_set_size_ &&
           k < max_k_)
    {
      AffineFlowMeasures inliers;
      inliers.clear();
      cv::Mat new_estimate = affineRANSAC(active_points, inliers,
                                          ransac_inlier_percent_est_,
                                          ransac_epsilon_, max_ransac_iters_);
      AffineFlowMeasure new_center;
      new_center.a = new_estimate;
      new_center.label = k;
      new_center.x = 0;
      new_center.y = 0;
      for (unsigned int i = 0; i < inliers.size(); ++i)
      {
        new_center.x += inliers[i].x;
        new_center.y += inliers[i].y;

        // Set labels for the removed points
        for (unsigned int j = 0; j < points.size(); ++j)
        {
          if (points[j] == inliers[i])
          {
            points[j].label = k;
          }
        }
        // Remove inliers from active points
        for (unsigned int j = 0; j < active_points.size(); )
        {
          if (inliers[i] == active_points[j])
          {
            active_points.erase(active_points.begin()+j);
          }
          else
          {
            ++j;
          }
        }
      }
      if (inliers.size() > 0)
      {
        new_center.x /= inliers.size();
        new_center.y /= inliers.size();
        cv::Mat V = new_center.a*new_center.X();
        new_center.u = V.at<float>(0,0);
        new_center.v = V.at<float>(1,0);
      }
      else
      {
        new_center.x = 0;
        new_center.y = 0;
        new_center.u = 0;
        new_center.v = 0;
      }
      cluster_centers.push_back(new_center);
      ROS_INFO_STREAM("Fit affine transform " << k << " with center ("
                      << new_center.x << ", " << new_center.y << ")");
      ROS_INFO_STREAM("Number of points remaining: " << active_points.size());
      ++k;
    }

#ifdef DISPLAY_FLOW_FIELD_CLUSTERING
    // ROS_INFO_STREAM("Displaying clusters");
    displayClusterCenters(cluster_centers, points, color_img);
    // ROS_INFO_STREAM("Displayed clusters");
#endif // DISPLAY_FLOW_FIELD_CLUSTERING

    return cluster_centers;
  }

  AffineFlowMeasures clusterSparseFlowKMeans(cv::Mat& color_img,
                                             cv::Mat& depth_img,
                                             cv::Mat& u, cv::Mat& v,
                                             cv::Mat& mask)
  {
    cv::Mat img_bw;
    cv::cvtColor(color_img, img_bw, CV_BGR2GRAY);
    // AffineFlowMeasures sparse_flow = ft_->updateTracks(img_bw, mask);
    AffineFlowMeasures sparse_flow = ft_->getMostRecentFlow();
    // TODO: Apply the mask to sparse_flow results

    // TODO: Try add in geometric matching here to complement the sparse feature
    // tracks
    for (unsigned int i = 0; i < sparse_flow.size(); ++i)
    {
      sparse_flow[i].a = estimateAffineTransform(u, v, sparse_flow[i].y,
                                                 sparse_flow[i].x,
                                                 affine_estimate_radius_);
    }
    AffineFlowMeasures cluster_centers;
    if (sparse_flow.size() < /*1*/ 2)
    {
      cluster_centers.clear();
      return cluster_centers;
    }
    cluster_centers = clusterAffineKMeans(color_img, u, v, sparse_flow);
    return cluster_centers;
  }

  AffineFlowMeasures clusterSparseFlowRANSAC(cv::Mat& color_img,
                                             cv::Mat& depth_img,
                                             cv::Mat& u, cv::Mat& v,
                                             cv::Mat& mask)
  {
    return clusterSparseFlowKMeans(color_img, depth_img, u, v, mask);
  }

  //
  // Core functions
  //

  AffineFlowMeasures clusterAffineKMeans(cv::Mat& color_img, cv::Mat& u,
                                         cv::Mat& v, AffineFlowMeasures& points)
  {
    const int num_samples = points.size();
    const int r_scale = color_img.cols / 2;

    const int num_sample_elements = 8;

    // Setup sample matrix for kmeans
    cv::Mat samples(num_samples, num_sample_elements, CV_32FC1);
    for (unsigned int i = 0; i < points.size(); ++i)
    {
      AffineFlowMeasure p = points[i];
      // TODO: This could be done better by reshaping p.a and setting it to a
      // submatrix of samples
      samples.at<float>(i, 0) = p.a.at<float>(0,0)*r_scale;
      samples.at<float>(i, 1) = p.a.at<float>(0,1)*r_scale;
      samples.at<float>(i, 2) = p.a.at<float>(0,2);
      samples.at<float>(i, 3) = p.a.at<float>(1,0)*r_scale;
      samples.at<float>(i, 4) = p.a.at<float>(1,1)*r_scale;
      samples.at<float>(i, 5) = p.a.at<float>(1,2);
      samples.at<float>(i, 6) = p.x;
      samples.at<float>(i, 7) = p.y;
    }

    std::vector<cv::Mat> labels;
    std::vector<cv::Mat> centers;
    double compactness[max_k_];
    cv::TermCriteria term_crit(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
                               kmeans_max_iter_, kmeans_epsilon_);

    AffineFlowMeasures cluster_centers;
    AffineFlowMeasures fewer_centers;
    AffineFlowMeasures best_centers;
    for (int K = 1; K <= min(max_k_, num_samples); ++K)
    {
      // Perform clustering with K centers
      cv::Mat labels_k;
      cv::Mat centers_k;
      double slack = cv::kmeans(samples, K, labels_k, term_crit,
                                kmeans_tries_, cv::KMEANS_PP_CENTERS,
                                centers_k);
      compactness[K-1] = slack;
      labels.push_back(labels_k);
      centers.push_back(centers_k);
      cluster_centers.clear();
      // Get descriptors for each cluster and compare them to the previous level
      for (int c = 0; c < K; ++c)
      {
        AffineFlowMeasure new_center;
        new_center.x = 0;
        new_center.y = 0;
        new_center.label = c;
        int num_members = 0;
        for (int i = 0; i < num_samples; ++i)
        {
          if (labels[K-1].at<uchar>(i,0) == c)
          {
            new_center.x += points[i].x;
            new_center.y += points[i].y;
            points[i].label = c;
            ++num_members;
          }
        }

        if (num_members <= 0 ||
            centers[K-1].cols == 0 || centers[K-1].rows == 0)
        {
          new_center.x = 0;
          new_center.y = 0;
          new_center.u = 0;
          new_center.v = 0;
        }
        else
        {
          new_center.x = new_center.x/num_members;
          new_center.y = new_center.y/num_members;

          // Correctly set the affine estimate
          // TODO: This could be done better by selecting a submatrix from centers
          // and then reshaping it
          new_center.a.create(2, 3, CV_32FC1);
          new_center.a.at<float>(0,0) = centers[K-1].at<float>(c,0) / r_scale;;
          new_center.a.at<float>(0,1) = centers[K-1].at<float>(c,1) / r_scale;;
          new_center.a.at<float>(0,2) = centers[K-1].at<float>(c,2);
          new_center.a.at<float>(1,0) = centers[K-1].at<float>(c,3) / r_scale;;
          new_center.a.at<float>(1,1) = centers[K-1].at<float>(c,4) / r_scale;;
          new_center.a.at<float>(1,2) = centers[K-1].at<float>(c,5);

          // Estimate flow of the cluster center using affine transform estimate
          cv::Mat V = new_center.a*new_center.X();
          new_center.u = V.at<float>(0,0);
          new_center.v = V.at<float>(1,0);
        }
        cluster_centers.push_back(new_center);
      }
      // Compare current K centers to centers of cardinality K-1
      if (K > 1)
      {
        float farthest_nearest_neighbor = 0;
        for (unsigned int i = 0; i < cluster_centers.size(); ++i)
        {
          float nn = FLT_MAX;
          for (unsigned int j = 0; j < fewer_centers.size(); ++j)
          {
            float dist = cluster_centers[i] - fewer_centers[j];
            if (dist < nn)
              nn = dist;
          }
          if (nn > farthest_nearest_neighbor)
            farthest_nearest_neighbor = nn;
        }
        // If no new clusters have center far enough from the current clusters,
        // Then we choose the previous set as best
        if (farthest_nearest_neighbor < minimum_new_cluster_separation_)
        {
          best_centers = fewer_centers;
          break;
        }
      }
      // Store current estimates for comparing in next iteration of K
      fewer_centers = cluster_centers;
      best_centers = cluster_centers;
    }
    ROS_INFO_STREAM("Chose " << best_centers.size() << " clusters");
#ifdef DISPLAY_FLOW_FIELD_CLUSTERING
    displayClusterCenters(best_centers, points, color_img, 0);
#endif // DISPLAY_FLOW_FIELD_CLUSTERING
    return best_centers;
  }

  cv::Mat estimateAffineTransform(cv::Mat& u, cv::Mat& v,
                                  const int r, const int c, const int radius)
  {
    const int r_min = max(r - radius, 0);
    const int r_max = min(r + radius, u.rows-1);
    const int c_min = max(c - radius, 0);
    const int c_max = min(c + radius, u.cols-1);
    const int r_range = r_max-r_min+1;
    const int c_range = c_max-c_min+1;
    const int num_eqs = r_range*c_range*2;
    cv::Mat a(6, 1, CV_32FC1, cv::Scalar(1.0));
    if (num_eqs < 6)
    {
      ROS_WARN_STREAM("Too few equations; num equations is: " << num_eqs);
      cv::Mat A = a.reshape(1, 2);
      return A;
    }
    cv::Mat phi(num_eqs, 6, CV_32FC1, cv::Scalar(0.0));
    cv::Mat V(num_eqs, 1, CV_32FC1, cv::Scalar(0.0));
    for (int r = r_min, out_row = 0; r <= r_max; ++r)
    {
      for (int c = c_min; c <= c_max; ++c, ++out_row)
      {
        phi.at<float>(out_row, 0) = r;
        phi.at<float>(out_row, 1) = c;
        phi.at<float>(out_row, 2) = 1.0;
        V.at<float>(out_row, 0) = u.at<float>(r,c);
        ++out_row;
        phi.at<float>(out_row, 3) = r;
        phi.at<float>(out_row, 4) = c;
        phi.at<float>(out_row, 5) = 1.0;
        V.at<float>(out_row, 0) = v.at<float>(r,c);
      }
    }
    try
    {
      cv::solve(phi, V, a, cv::DECOMP_SVD);
    }
    catch (cv::Exception cve)
    {
      ROS_ERROR_STREAM(cve.what());
    }
    cv::Mat A = a.reshape(1, 2);
    return A;
  }

  cv::Mat estimateAffineTransform(AffineFlowMeasures& points)
  {
    const int num_eqs = points.size()*2;
    cv::Mat phi(num_eqs, 6, CV_32FC1, cv::Scalar(0.0));
    cv::Mat V(num_eqs, 1, CV_32FC1, cv::Scalar(0.0));
    cv::Mat a(6, 1, CV_32FC1, cv::Scalar(1.0));
    if (num_eqs < 6)
    {
      ROS_WARN_STREAM("Too few equations; num equations is: " << num_eqs);
      cv::Mat A = a.reshape(1, 2);
      return A;
    }
    for (unsigned int i = 0, out_row = 0; i < points.size(); ++i, ++out_row)
    {
      AffineFlowMeasure p = points[i];
      phi.at<float>(out_row, 0) = p.y;
      phi.at<float>(out_row, 1) = p.x;
      phi.at<float>(out_row, 2) = 1.0;
      V.at<float>(out_row, 0) = p.u;
      ++out_row;
      phi.at<float>(out_row, 3) = p.y;
      phi.at<float>(out_row, 4) = p.x;
      phi.at<float>(out_row, 5) = 1.0;
      V.at<float>(out_row, 0) = p.v;
    }
    cv::solve(phi, V, a, cv::DECOMP_SVD);
    cv::Mat A = a.reshape(1, 2);
    return A;
  }

  /**
   * See how closely the current affine estimate matches the estimated flow
   *
   * @param f The flow estimate
   * @param A The affine region estimate
   *
   * @return Distortion score SSD between V and V_hat
   */
  float getAffineDistortion(AffineFlowMeasure f, cv::Mat A)
  {
    cv::Mat X(3, 1, CV_32FC1, cv::Scalar(1.0));
    X.at<float>(0,0) = f.x;
    X.at<float>(1,0) = f.y;

    cv::Mat V(2, 1, CV_32FC1, cv::Scalar(1.0));
    V.at<float>(0,0) = f.u;
    V.at<float>(0,0) = f.v;

    cv::Mat d = V - A*X;
    return d.at<float>(0,0)*d.at<float>(0,0)+d.at<float>(1,0)*d.at<float>(1,0);
  }

  /**
   * Determine the set of inliers for the current affine estimate
   *
   * @param points The set of all points given to RANSAC
   * @param cur_transform The current transform estimate
   * @param epsilon The threshold for acceptance as an inlier
   *
   * @return The set of points which score less than epsilon
   */
  AffineFlowMeasures getAffineConsensus(AffineFlowMeasures points,
                                        cv::Mat cur_transform, float epsilon)
  {
    AffineFlowMeasures inliers;
    for (unsigned int i = 0; i < points.size(); ++i)
    {
      float score = getAffineDistortion(points[i], cur_transform);
      if (score < epsilon)
      {
        inliers.push_back(points[i]);
      }
    }
    return inliers;
  }

  /**
   * Method fits an affine motion estimate to a set of image points using RANSAC
   *
   * @param points The set of individual flow estimates
   * @param inliers Returns the set of points determined to be inliers
   * @param inlier_percent_est the estimated percent of inliers in points
   * @param epsilon the threshold for being an inlier
   * @param max_iterations The maximum number of sample iterations to execute
   * @param min_iterations The minimum number of sample iterations to execute
   *
   * @return The best fit affine estimate
   */
  cv::Mat affineRANSAC(AffineFlowMeasures& points, AffineFlowMeasures& inliers,
                       float inlier_percent_est, float epsilon,
                       int max_iterations=0, int min_iterations = 2)
  {
    AffineFlowMeasures sample_points;
    AffineFlowMeasures cur_inliers;
    AffineFlowMeasures best_inliers;
    best_inliers.clear();
    bool done;
    int iter = 0;

    // Compute max_iterations as function of inlier percetage
    if (max_iterations == 0)
    {
      // Percent certainty
      const float p = 0.99;
      // Number of model parameters
      const float s = 3.0f;
      max_iterations = log(1 - p) / log(1-pow(inlier_percent_est, s));
    }

    while ( !done )
    {
      // Randomly select 3 points
      sample_points.clear();
      for (int i = 0; i < 3; ++i)
      {
        int r_idx = (rand() % (points.size()+1));
        sample_points.push_back(points[r_idx]);
      }
      // Estimate affine flow from them
      cv::Mat cur_transform = estimateAffineTransform(sample_points);
      cur_inliers = getAffineConsensus(points, cur_transform, epsilon);
      // Update best estimate if we have more points
      if ( best_inliers.size() < cur_inliers.size() )
      {
        best_inliers = cur_inliers;
      }
      // Check if sampling should stop
      iter++;
      done = ((iter > min_iterations &&
               (static_cast<float>(best_inliers.size())/points.size() >
                inlier_percent_est ||
                best_inliers.size() > inlier_percent_est*points.size() )) ||
               iter > max_iterations);
    }
    inliers = best_inliers;
    return estimateAffineTransform(inliers);
  }

  //
  // Helper Functions
  //

    /**
   * Display the results for the segmentation.
   *
   * @param cluster_centers The estimated segmentation centers
   * @param samples All points used in clustering
   * @param color_img The color image associated with the estimate
   */
  void displayClusterCenters(AffineFlowMeasures& cluster_centers,
                             AffineFlowMeasures& samples,
                             cv::Mat& color_img, int cur_max_k=0)
  {
    cv::Mat flow_center_disp = color_img.clone();
    for (unsigned int i = 0; i < cluster_centers.size(); ++i)
    {
      cv::Point pt_a(cluster_centers[i].x, cluster_centers[i].y);
      cv::Point pt_b(pt_a.x - cluster_centers[i].u,
                     pt_a.y - cluster_centers[i].v);
      cv::circle(flow_center_disp, pt_a, 20, cv::Scalar(0,255,0));
      cv::line(flow_center_disp, pt_a, pt_b, cv::Scalar(0,255,0));
    }
    std::stringstream center_disp_name;
    center_disp_name << "Flow Cluster Centers";
    if (cur_max_k != 0) center_disp_name << " " << cur_max_k;
    cv::imshow(center_disp_name.str(), flow_center_disp);
    cv::Mat flow_clusters_disp = color_img.clone();
    std::vector<cv::Scalar> colors;
    for (unsigned int k = 0; k < cluster_centers.size(); ++k)
    {
      cv::Scalar rand_color;
      rand_color[0] = (static_cast<float>(rand()) /
                       static_cast<float>(RAND_MAX))*255;
      rand_color[1] = (static_cast<float>(rand()) /
                       static_cast<float>(RAND_MAX))*255;
      rand_color[2] = (static_cast<float>(rand()) /
                       static_cast<float>(RAND_MAX))*255;
      colors.push_back(rand_color);
    }
    for (unsigned int i = 0; i < samples.size(); ++i)
    {
      AffineFlowMeasure s = samples[i];
      if (s.label < 0 || s.label > (cluster_centers.size()-1)) continue;
      if (std::sqrt(s.u*s.u+s.v*s.v) > 1.0)
      {
        cv::Scalar cur_color = colors[s.label];
        cv::line(flow_clusters_disp, cv::Point(s.x, s.y),
                 cv::Point(s.x - s.u, s.y - s.v), cur_color);
      }
    }
    std::stringstream cluster_disp_name;
    cluster_disp_name << "Flow Clusters";
    if (cur_max_k != 0) cluster_disp_name << " " << cur_max_k;
    cv::imshow(cluster_disp_name.str(), flow_clusters_disp);
  }

  //
  // Class member variables
  //
 protected:
  cv::Mat dx_kernel_;
  cv::Mat dy_kernel_;
  cv::Mat g_kernel_;
  FeatureTracker* ft_;
 public:
  int kmeans_max_iter_;
  double kmeans_epsilon_;
  double ransac_epsilon_;
  double ransac_inlier_percent_est_;
  int kmeans_tries_;
  int max_k_;
  int affine_estimate_radius_;
  int min_affine_point_set_size_;
  int max_ransac_iters_;
  double minimum_new_cluster_separation_;
};
