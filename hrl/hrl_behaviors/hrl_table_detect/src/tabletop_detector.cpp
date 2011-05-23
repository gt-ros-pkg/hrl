#include "hrl_table_detect/tabletop_detector.h"
using namespace std;

namespace hrl_table_detect {

    TabletopDetector::TabletopDetector() : img_trans(nh) {
    }

    
    void TabletopDetector::onInit() {
        ///////////////////// Parameters //////////////////////////////
        nh_priv = ros::NodeHandle("~");
        bool run_service;
        nh_priv.param<bool>("run_service", run_service, false);
        nh_priv.param<double>("scan_width", maxy, 1.5); miny = -maxy;
        nh_priv.param<double>("min_scan_depth", minx, 0.3);
        nh_priv.param<double>("max_scan_depth", maxx, 3.0);
        nh_priv.param<double>("min_table_height", minz, 0.5);
        nh_priv.param<double>("max_table_height", maxz, 1.0);
        nh_priv.param<double>("height_image_res", resolution, 200);
        nh_priv.param<double>("inlier_magnitude", inlier_magnitude, 200);
        nh_priv.param<int32_t>("num_edge_dilate", num_edge_dilate, 1);
        nh_priv.param<int32_t>("num_closes", num_closes, 1);
        nh_priv.param<double>("degree_bins", degree_bins, 0.3);
        nh_priv.param<double>("hough_thresh", hough_thresh, 30); hough_thresh *= resolution;
        nh_priv.param<double>("theta_gran", theta_gran, 1); theta_gran *= CV_PI/180;
        nh_priv.param<double>("rho_gran", rho_gran, 0.1); rho_gran *= resolution;
        nh_priv.param<double>("xgran", xgran, 0.1); xgran *= resolution;
        nh_priv.param<double>("ygran", ygran, 0.1); ygran *= resolution;
        imgx = (maxx-minx)*resolution;
        imgy = (maxy-miny)*resolution;
        ///////////////////////////////////////////////////////////////

        grasp_points_found = false;
        height_img_sum = cv::Mat::zeros(imgx, imgy, CV_32F);
        height_img_count = cv::Mat::zeros(imgx, imgy, CV_32F);
        height_img_max = cv::Mat::zeros(imgx, imgy, CV_8U);

        pc_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/table_detection_vis", 1);
        height_pub = img_trans.advertise("/height_image", 1);
        pose_arr_pub = nh.advertise<geometry_msgs::PoseArray>("/table_approach_poses", 1);
        nh_priv = ros::NodeHandle("~");
        if(run_service) {
            table_detect_start = nh.advertiseService("/table_detect_start", &TabletopDetector::startCallback, this);
            table_detect_stop = nh.advertiseService("/table_detect_stop", &TabletopDetector::stopCallback, this);
            table_detect_inst = nh.advertiseService("/table_detect_inst", &TabletopDetector::instCallback, this);
        }
        else
            pc_sub = nh.subscribe("/kinect_head/rgb/points", 1, &TabletopDetector::pcCallback, this);
        ros::Duration(1.0).sleep();
    }

    bool TabletopDetector::startCallback(hrl_table_detect::DetectTableStart::Request& req, 
                                       hrl_table_detect::DetectTableStart::Response& resp) {
        grasp_points_found = false;
        height_img_sum = cv::Mat::zeros(imgx, imgy, CV_32F);
        height_img_count = cv::Mat::zeros(imgx, imgy, CV_32F);
        height_img_max = cv::Mat::zeros(imgx, imgy, CV_8U);
        pc_sub = nh.subscribe("/kinect_head/rgb/points", 1, &TabletopDetector::pcCallback, this);
        return true;
    }

    bool TabletopDetector::stopCallback(hrl_table_detect::DetectTableStop::Request& req, 
                                       hrl_table_detect::DetectTableStop::Response& resp) {
        pc_sub.shutdown();
        resp.grasp_points = grasp_points;
        return true;
    }

    bool TabletopDetector::instCallback(hrl_table_detect::DetectTableInst::Request& req, 
                                       hrl_table_detect::DetectTableInst::Response& resp) {
        grasp_points_found = false;
        height_img_sum = cv::Mat::zeros(imgx, imgy, CV_32F);
        height_img_count = cv::Mat::zeros(imgx, imgy, CV_32F);
        height_img_max = cv::Mat::zeros(imgx, imgy, CV_8U);
        pc_sub = nh.subscribe("/kinect_head/rgb/points", 1, &TabletopDetector::pcCallback, this);
        double begin_time = ros::Time::now().toSec();
        double now = begin_time;
        ros::Rate r(100);
        while(ros::ok() && now - begin_time < req.block_time) {
            ros::spinOnce();
            now = ros::Time::now().toSec();
            r.sleep();
        }
        pc_sub.shutdown();
        resp.grasp_points = grasp_points;
        return true;
    }

    bool compind(uint32_t a, uint32_t b, vector<float> v) { return v[a] < v[b]; }

    void convCvBox2DPoly(CvBox2D& box, CvPoint**& pts, int*& npts) {
        pts = new CvPoint*[1]; pts[0] = new CvPoint[4];
        float x = box.center.x, y = box.center.y;
        float x1 = box.size.height*cos(box.angle-CV_PI/2), y1 = -box.size.height*sin(box.angle-CV_PI/2);
        float x2 = box.size.width*cos(box.angle), y2 = -box.size.width*sin(box.angle);
        pts[0][0].x = x+x1+x2; pts[0][0].y = y+y1+y2;
        pts[0][1].x = x-x1+x2; pts[0][1].y = y-y1+y2;
        pts[0][2].x = x-x1-x2; pts[0][2].y = y-y1-y2;
        pts[0][3].x = x+x1-x2; pts[0][3].y = y+y1-y2;
        pts[0][4].x = x+x1+x2; pts[0][4].y = y+y1+y2;
        npts = new int[1]; npts[0] = 5;
    }

    void drawCvBox2D(IplImage& img, CvBox2D& box, cv::Scalar color, int width=1) {
        CvPoint** rpts; int* npts;
        convCvBox2DPoly(box, rpts, npts);
        cvPolyLine(&img, rpts, npts, 1, 1, color, width);
        delete[] rpts[0]; delete[] rpts; delete[] npts;
    }

    void fillCvBox2D(IplImage& img, CvBox2D& box, cv::Scalar color) {
        CvPoint** rpts; int* npts;
        convCvBox2DPoly(box, rpts, npts);
        cvFillPoly(&img, rpts, npts, 1, color);
        delete[] rpts[0]; delete[] rpts; delete[] npts;
    }

    void TabletopDetector::pcCallback(sensor_msgs::PointCloud2::ConstPtr pc_msg) {
        if(!pc_lock.try_lock())
            return;

        pcl::PointCloud<pcl::PointXYZRGB> pc_full, pc_full_frame;
        pcl::fromROSMsg(*pc_msg, pc_full);
        string base_frame("/base_link");
        ros::Time now = ros::Time::now();
        tf_listener.waitForTransform(pc_msg->header.frame_id, base_frame, now, ros::Duration(3.0));
        pcl_ros::transformPointCloud(base_frame, pc_full, pc_full_frame, tf_listener);
        // pc_full_frame is in torso lift frame

        cv::Mat cur_height_img = cv::Mat::zeros(imgx, imgy, CV_8U);
        
        BOOST_FOREACH(const pcl::PointXYZRGB& pt, pc_full_frame.points) {
            if(pt.x != pt.x || pt.y != pt.y || pt.z != pt.z)
                continue;
            int32_t x, y, z;
            x = (pt.x - minx)/(maxx-minx) * imgx;
            y = (pt.y - miny)/(maxy-miny) * imgy;
            z = (pt.z - minz)/(maxz-minz) * 256;
            if(x < 0 || y < 0) continue; 
            if(x >= imgx || y >= imgy) continue;
            if(z < 0 || z >= 256) continue;
            if(cur_height_img.at<uint8_t>(x, y) == 0 || cur_height_img.at<uint8_t>(x, y) < (uint8_t) z)
                cur_height_img.at<uint8_t>(x, y) = (uint8_t) z;
        }
        cv::max(height_img_max, cur_height_img, height_img_max);
        cv::Mat cur_height_img_flt;
        cur_height_img.convertTo(cur_height_img_flt, CV_32F);
        height_img_sum += cur_height_img_flt;
        cv::Mat cur_count(imgx, imgy, CV_8U);
        cur_count = (cur_height_img > 0) / 255;
        cv::Mat cur_count_flt(imgx, imgy, CV_32F);
        cur_count.convertTo(cur_count_flt, CV_32F);
        height_img_count += cur_count_flt;
        cv::Mat height_img_avg_flt = height_img_sum / height_img_count;
        cv::Mat height_img_avg(imgx, imgy, CV_8U);
        height_img_avg_flt.convertTo(height_img_avg, CV_8U);
        height_img_avg = height_img_max;

        cv::Mat height_hist(256, 1, CV_32F, cv::Scalar(0));
        for(uint32_t x=0;x<imgx;x++)
            for(uint32_t y=0;y<imgy;y++) {
                if(height_img_avg.at<uint8_t>(x,y) == 255)
                    height_img_avg.at<uint8_t>(x,y) = 0;
                if(height_img_avg.at<uint8_t>(x,y) != 0) {
                    height_hist.at<float>(height_img_avg.at<uint8_t>(x,y), 0)++;
                }
            }
        ////////////////////// Finding best table height /////////////////////////
        uint32_t gfiltlen = 25;
        float stddev = 256/(maxz-minz) * 0.015;
        cv::Mat gauss_filt(gfiltlen, 1, CV_32F, cv::Scalar(0));
        for(uint32_t i=0;i<gfiltlen;i++)
            gauss_filt.at<float>(i,0) = 0.39894 / stddev * std::exp(-(i-((float)gfiltlen)/2)*(i-((float)gfiltlen)/2)/(2*stddev*stddev));
        //cout << gauss_filt;
        uint32_t maxval = 0, maxidx = 0;
        for(uint32_t i=0;i<256-gfiltlen;i++) {
            uint32_t sum = 0;
            for(uint32_t j=0;j<gfiltlen;j++) 
                sum += height_hist.at<float>(i+j,0) * gauss_filt.at<float>(j,0);
            if(sum > maxval && i != 0) {
                maxval = sum;
                maxidx = i+gfiltlen/2;
            }
        }
        int32_t table_height = ((int32_t)maxidx);
        //printf("%d %d, ", maxval, maxidx);
        /////////////////////////// Getting table binary /////////////////////
        cv::Mat height_img_thresh(imgx, imgy, CV_8U);
        height_img_thresh = height_img_avg.clone();
        for(uint32_t x=0;x<imgx;x++)
            for(uint32_t y=0;y<imgy;y++) {
                if(std::fabs(table_height - ((int32_t)height_img_thresh.at<uint8_t>(x,y))) < stddev*2)
                    height_img_thresh.at<uint8_t>(x,y) = 255;
                else
                    height_img_thresh.at<uint8_t>(x,y) = 0;
            }
        //////////////////////////////////////////////////////////////////
        IplImage height_img_thresh_ipl = height_img_thresh;
        IplConvKernel* element = cvCreateStructuringElementEx(3, 3, 1, 1, CV_SHAPE_RECT);
        cvMorphologyEx(&height_img_thresh_ipl, &height_img_thresh_ipl, NULL, element, CV_MOP_CLOSE, num_closes);
        //cvMorphologyEx(&height_img_thresh, &height_img_thresh, NULL, element, CV_MOP_OPEN, 2);

        cv::Mat height_img_thresh_blob = height_img_thresh.clone();
        IplImage blob_img = height_img_thresh_blob;
        CBlobResult blobs = CBlobResult(&blob_img, NULL, 0);
        //blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, 10);
        CBlob biggestblob;
        blobs.GetNthBlob(CBlobGetArea(), 0, biggestblob);
        cv::Mat table_blob(imgx, imgy, CV_8U, cv::Scalar(0)); IplImage table_blob_img = table_blob;
        biggestblob.FillBlob(&table_blob_img, cv::Scalar(150));
        //drawCvBox2D(blob_img, table_roi, cv::Scalar(50), 1);
        CvBox2D table_roi = biggestblob.GetEllipse(); table_roi.angle *= CV_PI/180;
        cv::Mat table_hull(imgx, imgy, CV_8U, cv::Scalar(0));
        IplImage hull_img = table_hull;
        fillCvBox2D(hull_img, table_roi, cv::Scalar(255));
        //printf("Cvbox: %f, %f, %f, %f, %f\n", table_roi.center.x, table_roi.center.y, table_roi.size.width, table_roi.size.height, table_roi.angle);

        //cv::Mat height_morph(imgx, imgy, CV_8U, cv::Scalar(0));
        //cv::Mat tmp_img(imgx, imgy, CV_8U, cv::Scalar(0));
        //IplImage t1 = height_img_thresh; IplImage  = height_morph;

        cv::Mat table_edge(imgx, imgy, CV_8U);
        cv::Sobel(table_blob, table_edge, CV_8U, 0, 1, 1);

        cv::Mat above_table(imgx, imgy, CV_8U);
        bitwise_and(height_img_max > table_height + stddev*2, table_hull, above_table);
        IplImage above_table_img = above_table;

        CBlobResult obj_blobs = CBlobResult(&above_table_img, NULL, 0);
        CBlob biggest_obj_blob;
        double objcentx = 0, objcenty = 0;
        if(obj_blobs.GetNumBlobs() > 0) {
            obj_blobs.GetNthBlob(CBlobGetArea(), 0, biggest_obj_blob);
            CvBox2D obj_box = biggest_obj_blob.GetEllipse();
            objcenty = obj_box.center.x, objcentx = obj_box.center.y;
        }
        //double objcentx = 0, objcenty = 0;

        //cv::Mat table_edge = height_morph.clone();
        //cvMorphologyEx(&height_img, &height_morph, NULL, element, CV_MOP_CLOSE);

        //cvFillPoly(&ipl_hull, rpts, npts, 1, cv::Scalar(255));

        //cv::Mat obj_img(imgx, imgy, CV_8U, cv::Scalar(0));
        //std::vector<int32_t> xfeats, yfeats, zfeats;
        //double sumobjx = 0, sumobjy = 0, sumobja = 0;
        //for(uint32_t y=0;y<imgx;y++) 
        //    for(uint32_t x=0;x<imgx;x++) 
        //        if(table_hull.at<uint8_t>(x,y) == 255 && height_morph.at<uint8_t>(x,y) == 0
        //                && height_img_avg.at<uint8_t>(x,y) > table_height + stddev*2) {
        //            obj_img.at<uint8_t>(x,y) = height_img_avg.at<uint8_t>(x,y);
        //            sumobjx += x; sumobjy += y; sumobja ++;
        //            //xfeats.push_back(x); yfeats.push_back(y); 
        //            //zfeats.push_back(height_img.at<uint8_t>(x,y));
        //        }
        //double objcentx = sumobjx/sumobja, objcenty = sumobjy/sumobja;

        CvMemStorage* storage = cvCreateMemStorage(0);
        CvSeq* lines = 0;
        cv::Mat lines_img = height_img_max.clone();
        IplImage lines_img_ipl = lines_img;
        IplImage table_edge_ipl = table_edge;
        cvMorphologyEx(&table_edge_ipl, &table_edge_ipl, NULL, element, CV_MOP_DILATE, num_edge_dilate);
        lines = cvHoughLines2(&table_edge_ipl, storage, CV_HOUGH_STANDARD, 1, degree_bins*CV_PI/180, hough_thresh, 0, 0);
        vector<float> theta_bins, rho_bins;
        vector<uint32_t> count_bins;
        for(uint32_t i=0; i < (uint32_t) lines->total; i++) {
            float* line = (float*)cvGetSeqElem(lines, i);
            float rho = line[0];
            float theta = line[1];
            bool found_same = false;
            for(int32_t j=theta_bins.size()-1; j >= 0; j--) {
                if(fabs(theta_bins[j]/count_bins[j] - theta) < theta_gran &&
                   fabs(rho_bins[j]/count_bins[j] - rho) < rho_gran) {
                    theta_bins[j] += theta;
                    rho_bins[j] += rho;
                    count_bins[j]++;
                    found_same = true;
                    break;
                }
            }
            if(!found_same) {
                theta_bins.push_back(theta);
                rho_bins.push_back(rho);
                count_bins.push_back(1);
            }

            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            CvPoint pt1, pt2;
            a = cos(theta); b = sin(theta);
            //a = cos(theta+CV_PI/2); b = sin(theta+CV_PI/2);
            //x0 = objcenty; y0 = objcentx;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            cvLine(&lines_img_ipl, pt1, pt2, cv::Scalar(100), 2, 8 );
        }
        //delete[] lines;
        for(uint32_t i=0;i<theta_bins.size();i++) {
            theta_bins[i] /= count_bins[i];
            rho_bins[i] /= count_bins[i];
        }
        vector<float> posesx, posesy, poses_theta, dists_obj;
        vector<uint32_t> pose_inds;
        for(uint32_t i=0;i<theta_bins.size();i++) {
            double theta = theta_bins[i];
            double rho = rho_bins[i];
            double a1 = cos(theta-CV_PI/2), b1 = sin(theta-CV_PI/2);
            double a2 = cos(theta-CV_PI), b2 = sin(theta-CV_PI);
            double vvcl = a2*b1-b2*a1, deltpx = cos(theta)*rho-objcenty, deltpy = sin(theta)*rho-objcentx;
            double pvcr = deltpx*b1 - deltpy*a1;
            double t = pvcr/vvcl;
            double posey = objcenty + t*a2, posex = objcentx + t*b2;
            printf("\naPose %d: (t: %f, %f, %f)[%f, %f](%f, %f)[%f, %f](1 %f, %f)(2 %f, %f)[theta %f, %f]\n", i, t, posex, posey, t*a2, t*b2, a1*rho, b1*rho, objcentx, objcenty, a1, b1, a2, b2, theta, rho);
            if(posex == posex && posey == posey &&
                posex >= 0 && posey >= 0 &&
                posex < imgx && posey < imgy) {
                posesx.push_back(posex); posesy.push_back(posey); poses_theta.push_back(theta);
                pose_inds.push_back(posesx.size()-1);
                float dist = (posex-objcentx)*(posex-objcentx)+(posey-objcenty)*(posey-objcenty);
                dists_obj.push_back(dist);
            }
            //lines_img.at<uint8_t>(posex, posey)
        }
        boost::function<bool(uint32_t, uint32_t)> sortind = boost::bind(&compind, _1, _2, dists_obj);
        sort(pose_inds.begin(), pose_inds.end(), sortind);

        vector<float> retposesx, retposesy, retposesr;
        grasp_points.poses.clear();
        for(uint32_t i=0;i<pose_inds.size();i++) {
            float posex = posesx[pose_inds[i]], posey = posesy[pose_inds[i]];
            float poser = -poses_theta[pose_inds[i]] + 3*CV_PI/2;
            bool same_found = false;
            for(int32_t j=((int)retposesx.size())-1;j>=0;j--) {
                if(fabs(posex - retposesx[j]) < xgran && 
                   fabs(posey - retposesy[j]) < ygran) {
                    same_found = true;
                } 
            }
            if(!same_found) {
                retposesx.push_back(posex);
                retposesy.push_back(posey);
                retposesr.push_back(poser);
                geometry_msgs::Pose cpose;
                cpose.position.x = posex/imgx*(maxx-minx) + minx;
                cpose.position.y = posey/imgy*(maxy-miny) + miny;
                cpose.position.z = table_height/256.0*(maxz-minz) + minz;
                btMatrix3x3 quatmat; btQuaternion quat;
                quatmat.setEulerZYX(poser, 0 , 0);
                quatmat.getRotation(quat);
                cpose.orientation.x = quat.x(); cpose.orientation.y = quat.y();
                cpose.orientation.z = quat.z(); cpose.orientation.w = quat.w();
                grasp_points.poses.push_back(cpose);
            }
        }
        grasp_points.header.stamp = ros::Time::now();
        grasp_points.header.frame_id = base_frame;
        pose_arr_pub.publish(grasp_points);
        

        printf("\nCenter (%f, %f)\n", objcentx, objcenty);
        for(uint32_t i=0;i<retposesx.size();i++) {
        //for(uint32_t i=0;i<1;i++) {
            printf("\nPose %d: (%f, %f, r: %f)\n", i, retposesx[i], retposesy[i], retposesr[i]);
            //CvPoint centerpt; centerpt.x = objcenty; centerpt.y = objcentx;
            CvPoint centerpt; centerpt.x = retposesy[i]; centerpt.y = retposesx[i];
            cvCircle(&lines_img_ipl, centerpt, 3, cv::Scalar(200), 2);
        }


        
        //cv::Mat obj_feats(xfeats.size(), 1, CV_32S, cv::Scalar(0));
        //for(uint32_t i=0;i<xfeats.size();i++) {
        //    obj_feats.at<uint32_t>(i,0) = xfeats[i]; obj_feats.at<uint32_t>(i,1) = yfeats[i]; obj_feats.at<uint32_t>(i,2) = zfeats[i]; 
        //}
        //cvflann::KMeansIndexParams kmips;
        //kmips.branching = 32;
        //kmips.iterations = 11;
        //kmips.centers_init = cvflann::CENTERS_RANDOM;
        //kmips.cb_index = 0.2;
        //cv::Mat obj_centers;
        //CvMat obj_feats_mat = obj_feats;
        ////cvflann::Matrix<uint32_t> obj_feats_mat;
        ////cvflann::Matrix<cvflann::EUCLIDEAN> obj_centers_mat;
        //int num_clust = cvflann::hierarchicalClustering<CV_32S,CV_32S>(obj_feats_mat, obj_centers, kmips);
        //printf("\nNum clust: %d \n", num_clust);
        


        cv::Mat table_edge2 = table_edge.clone();
        IplImage table_edge_ipl2 = table_edge2;
        cvMorphologyEx(&table_edge_ipl2, &table_edge_ipl2, NULL, element, CV_MOP_DILATE, 3);

        BOOST_FOREACH(const pcl::PointXYZRGB& pt, pc_full_frame.points) {
            if(pt.x != pt.x || pt.y != pt.y || pt.z != pt.z)
                continue;
            int32_t x, y, z;
            x = (pt.x - minx)/(maxx-minx) * imgx;
            y = (pt.y - miny)/(maxy-miny) * imgy;
            z = (pt.z - minz)/(maxz-minz) * 256;
            if(x < 0 || y < 0) continue; 
            if(x >= imgx || y >= imgy) continue;
            if(z < 0 || z >= 256) continue;
            if(table_blob.at<uint8_t>(x,y) == 255 && 
                    std::fabs(table_height - z) < stddev*2) {
                uint32_t red = 0xFFFF0000;
                ((uint32_t*) &pt.rgb)[0] = red;
            }
            if(table_edge2.at<uint8_t>(x,y) == 255 && 
                    std::fabs(table_height - z) < stddev*4) {
                uint32_t blue = 0xFF0000FF;
                ((uint32_t*) &pt.rgb)[0] = blue;
            }
        }

        cv_bridge::CvImage cvb_height_img;
        //cvb_height_img.image = height_img_avg;
        //cvb_height_img.image = height_img_max;
        //cvb_height_img.image = height_morph;
        //cvb_height_img.image = obj_img;
        cvb_height_img.image = lines_img;
        //cvb_height_img.image = height_img_thresh_blob;
        //cvb_height_img.image = table_blob;
        //cvb_height_img.image = height_img_thresh;
        //cvb_height_img.image = above_table;
        //cvb_height_img.image = table_edge;
        cvb_height_img.header.stamp = ros::Time::now();
        cvb_height_img.header.frame_id = base_frame;
        cvb_height_img.encoding = enc::MONO8;
        height_pub.publish(cvb_height_img.toImageMsg());
        pc_full_frame.header.stamp = ros::Time::now();
        pc_full_frame.header.frame_id = base_frame;
        pc_pub.publish(pc_full_frame);

        if(grasp_points.poses.size() > 0)
            grasp_points_found = true;

        //delete element;
        pc_lock.unlock();
    }

};

using namespace hrl_table_detect;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tabletop_approach");
    TabletopDetector ta;
    ta.onInit();
    ros::spin();
    return 0;
}
