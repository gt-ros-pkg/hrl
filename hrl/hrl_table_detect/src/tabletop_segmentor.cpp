#include "hrl_table_detect/tabletop_segmentor.h"
using namespace std;

namespace hrl_table_detect {

    TabletopSegmentor::TabletopSegmentor() : img_trans(nh) {
    }

    
    void TabletopSegmentor::onInit() {
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
        nh_priv.param<double>("obj_min_area", obj_min_area, 0.02); obj_min_area *= resolution;
        imgx = (maxx-minx)*resolution;
        imgy = (maxy-miny)*resolution;
        ///////////////////////////////////////////////////////////////

        obj_poses_found = false;
        height_img_sum = cv::Mat::zeros(imgx, imgy, CV_32F);
        height_img_count = cv::Mat::zeros(imgx, imgy, CV_32F);
        height_img_max = cv::Mat::zeros(imgx, imgy, CV_8U);

        //pc_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/table_detection_vis", 1);
        height_pub = img_trans.advertise("/obj_height_image", 1);
        obj_arr_pub = nh.advertise<geometry_msgs::PoseArray>("/table_object_poses", 1);
        nh_priv = ros::NodeHandle("~");
        if(run_service) {
            table_detect_start = nh.advertiseService("/obj_segment_start", &TabletopSegmentor::startCallback, this);
            table_detect_stop = nh.advertiseService("/obj_segment_stop", &TabletopSegmentor::stopCallback, this);
            table_detect_inst = nh.advertiseService("/obj_segment_inst", &TabletopSegmentor::instCallback, this);
        }
        else
            pc_sub = nh.subscribe("/kinect_head/rgb/points", 1, &TabletopSegmentor::pcCallback, this);
        ros::Duration(1.0).sleep();
    }

    bool TabletopSegmentor::startCallback(hrl_table_detect::DetectTableStart::Request& req, 
                                       hrl_table_detect::DetectTableStart::Response& resp) {
        obj_poses_found = false;
        height_img_sum = cv::Mat::zeros(imgx, imgy, CV_32F);
        height_img_count = cv::Mat::zeros(imgx, imgy, CV_32F);
        height_img_max = cv::Mat::zeros(imgx, imgy, CV_8U);
        pc_sub = nh.subscribe("/kinect_head/rgb/points", 1, &TabletopSegmentor::pcCallback, this);
        return true;
    }

    bool TabletopSegmentor::stopCallback(hrl_table_detect::DetectTableStop::Request& req, 
                                       hrl_table_detect::DetectTableStop::Response& resp) {
        pc_sub.shutdown();
        resp.grasp_points = obj_poses;
        return true;
    }

    bool TabletopSegmentor::instCallback(hrl_table_detect::DetectTableInst::Request& req, 
                                       hrl_table_detect::DetectTableInst::Response& resp) {
        obj_poses_found = false;
        height_img_sum = cv::Mat::zeros(imgx, imgy, CV_32F);
        height_img_count = cv::Mat::zeros(imgx, imgy, CV_32F);
        height_img_max = cv::Mat::zeros(imgx, imgy, CV_8U);
        pc_sub = nh.subscribe("/kinect_head/rgb/points", 1, &TabletopSegmentor::pcCallback, this);
        double begin_time = ros::Time::now().toSec();
        double now = begin_time;
        ros::Rate r(100);
        while(ros::ok() && now - begin_time < req.block_time) {
            ros::spinOnce();
            now = ros::Time::now().toSec();
            r.sleep();
        }
        pc_sub.shutdown();
        resp.grasp_points = obj_poses;
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

    void TabletopSegmentor::pcCallback(sensor_msgs::PointCloud2::ConstPtr pc_msg) {
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

        cv::Mat above_table(imgx, imgy, CV_8U);
        bitwise_and(height_img_max > table_height + stddev*2, table_hull, above_table);
        IplImage above_table_img = above_table;

        CBlobResult obj_blobs = CBlobResult(&above_table_img, NULL, 0);
        obj_blobs.Filter(obj_blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, obj_min_area);
        CBlob cur_obj_blob;
        vector<float> obj_cents_x, obj_cents_y, obj_cents_r, obj_areas, obj_dists;
        vector<uint32_t> obj_inds;
        for(int i=0;i<obj_blobs.GetNumBlobs();i++) {
            obj_blobs.GetNthBlob(CBlobGetArea(), i, cur_obj_blob);
            CvBox2D obj_box = cur_obj_blob.GetEllipse();
            obj_cents_x.push_back(obj_box.center.x);
            obj_cents_y.push_back(obj_box.center.y);
            obj_cents_r.push_back(obj_box.angle * CV_PI/180);
            obj_areas.push_back(cur_obj_blob.Area());
            obj_dists.push_back((obj_box.center.x-imgx/2)*(obj_box.center.x-imgx/2)+obj_box.center.y*obj_box.center.y);
            obj_inds.push_back(i);
        }
        boost::function<bool(uint32_t, uint32_t)> sortind = boost::bind(&compind, _1, _2, obj_dists);
        sort(obj_inds.begin(), obj_inds.end(), sortind);

        obj_poses.poses.clear();
        for(uint32_t i=0;i<obj_inds.size();i++) {
            float posey = obj_cents_x[obj_inds[i]], posex = obj_cents_y[obj_inds[i]];
            float poser = -obj_cents_r[obj_inds[i]] + 3*CV_PI/2;
            geometry_msgs::Pose cpose;
            cpose.position.x = posex/imgx*(maxx-minx) + minx;
            cpose.position.y = posey/imgy*(maxy-miny) + miny;
            cpose.position.z = table_height/256.0*(maxz-minz) + minz;
            btMatrix3x3 quatmat; btQuaternion quat;
            quatmat.setEulerZYX(poser, 0 , 0);
            quatmat.getRotation(quat);
            cpose.orientation.x = quat.x(); cpose.orientation.y = quat.y();
            cpose.orientation.z = quat.z(); cpose.orientation.w = quat.w();
            CvPoint centerpt; centerpt.y = posex; centerpt.x = posey;
            printf("[%d](%f, %f, area: %f)\n", i, posex, posey, obj_areas[obj_inds[i]]);
            IplImage height_img_max_ipl = height_img_max;
            cvCircle(&height_img_max_ipl, centerpt, 3, cv::Scalar(200), 2);
            obj_poses.poses.push_back(cpose);
        }
        obj_poses.header.stamp = ros::Time::now();
        obj_poses.header.frame_id = base_frame;
        obj_arr_pub.publish(obj_poses);

        cv_bridge::CvImage cvb_height_img;
        //cvb_height_img.image = height_img_avg;
        //cvb_height_img.image = height_img_max;
        //cvb_height_img.image = height_morph;
        //cvb_height_img.image = obj_img;
        //cvb_height_img.image = height_img_thresh_blob;
        //cvb_height_img.image = table_blob;
        //cvb_height_img.image = height_img_thresh;
        cvb_height_img.image = above_table;
        //cvb_height_img.image = table_edge;
        cvb_height_img.header.stamp = ros::Time::now();
        cvb_height_img.header.frame_id = base_frame;
        cvb_height_img.encoding = enc::MONO8;
        height_pub.publish(cvb_height_img.toImageMsg());
        pc_full_frame.header.stamp = ros::Time::now();
        pc_full_frame.header.frame_id = base_frame;
        //pc_pub.publish(pc_full_frame);

        if(obj_poses.poses.size() > 0)
            obj_poses_found = true;

        //delete element;
        pc_lock.unlock();
    }

};

using namespace hrl_table_detect;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tabletop_segmentation");
    TabletopSegmentor ta;
    ta.onInit();
    ros::spin();
    return 0;
}
