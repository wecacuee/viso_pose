#include <fstream>
#include <string>
#include<iostream>
#include<iomanip>
#include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
// #include "semslam_msgs/ObjectDetection.h"

#include <typeinfo>
#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// #include <boost/date_time/posix_time/posix_time.hpp>

void
subscriberConnectedCallback(const ros::SingleSubscriberPublisher& ssp)
{
    ROS_WARN_STREAM("[KITTI Data] Subscriber " << ssp.getSubscriberName()
                                               << " connected to topic "
                                               << ssp.getTopic() << ".");
}

void
imageSubscriberConnectedCallback(
  const image_transport::SingleSubscriberPublisher& ssp)
{
    ROS_WARN_STREAM("[KITTI Data] Subscriber " << ssp.getSubscriberName()
                                               << " connected to topic "
                                               << ssp.getTopic() << ".");
}

int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "kitti_data");
    ros::NodeHandle nh;

    ros::NodeHandle data_nh("~");

    image_transport::ImageTransport it(nh);

    boost::function<void(const image_transport::SingleSubscriberPublisher&)>
      image_fn = imageSubscriberConnectedCallback;

    image_transport::Publisher cam0_pub =
      it.advertise("cam0/image_raw",
                   1000,
                   (image_transport::SubscriberStatusCallback)
                     imageSubscriberConnectedCallback);
    ros::Publisher cam0_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>("cam0/camera_info", 10);

    image_transport::Publisher cam1_pub =
      it.advertise("cam1/image_raw", 1000, image_fn, image_fn);
    ros::Publisher cam1_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>("cam1/camera_info", 10);

    // ros::Publisher car_det_pub =
    // nh.advertise<semslam_msgs::ObjectDetection>("/object_detector/car/detections",
    // 1000, (ros::SubscriberStatusCallback)subscriberConnectedCallback);
    // ros::Publisher window_det_pub =
    // nh.advertise<semslam_msgs::ObjectDetection>("/object_detector/window/detections",
    // 1000);

    // ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu0", 1000);

    // delay
    double delay_secs = 1.0;
    data_nh.getParam("delay", delay_secs);

    std::string kitti_dir;
    data_nh.getParam("data_dir", kitti_dir);

    ROS_INFO_STREAM("KITTI cam0 has " << cam0_pub.getNumSubscribers()
                                      << " subscribers.");

    ROS_INFO_STREAM("Waiting for " << delay_secs
                                   << " seconds before publishing data.");
    ros::Duration(delay_secs).sleep();

    ROS_INFO_STREAM("KITTI cam0 has " << cam0_pub.getNumSubscribers()
                                      << " subscribers.");

    double rate;
    if (!data_nh.getParam("rate", rate)) {
        rate = 1.0;
    }

    double t_start, t_end;

    if (!data_nh.getParam("t_start", t_start)) {
        t_start = 0;
    }

    if (!data_nh.getParam("t_end", t_end)) {
        t_end = std::numeric_limits<double>::infinity();
    }

    // double t_start = 11.8;
    // double t_end = 13.3;

    // std::string
    // kitti_dir("/home/sean/data/raw_data/2011_09_30/2011_09_30_drive_0020_sync");
    // std::string
    // kitti_dir("/home/sean/data/raw_data/2011_09_26/2011_09_26_drive_0014_sync");
    // std::string kitti_dir("/home/sean/data/kitti/sequences/05");
    // std::string kitti_dir("/media/sean/My
    // Book/ubuntu_home/data/dataset/sequences/05"); std::string
    // kitti_dir("/media/sean/My Book/ubuntu_home/data/dataset/sequences/05");
    // std::string kitti_dir("/media/sean/My
    // Book/ubuntu_home/data/dataset/sequences/07");

    std::string cam_ts_filename = kitti_dir + "/times.txt";
    std::ifstream cam_ts_file(cam_ts_filename);

    if (!cam_ts_file) {
        ROS_ERROR_STREAM("unable to open file " << cam_ts_filename);
        exit(1);
    }

    // read camera times
    // std::vector<ros::Time> cam_ts;
    std::vector<double> cam_ts;
    while (cam_ts_file.good()) {
        double t;
        cam_ts_file >> t;
        cam_ts.push_back(t / rate);

        /*
        std::string ts;
        std::getline(cam_ts_file, ts);

        if (ts.length() > 0) {
                ros::Time t_ros =
        ros::Time::fromBoost(boost::posix_time::time_from_string(ts));
                cam_ts.push_back(t_ros);
        }
        */
    }

    ROS_INFO_STREAM(
      cam_ts.size()
      << " camera msmts"); // and " << imu_ts.size() << " imu msmts.");

    // build camera info msg
    sensor_msgs::CameraInfo info0_msg;
    sensor_msgs::CameraInfo info1_msg;

    info0_msg.height = 370;
    info0_msg.width = 1226;
    info0_msg.distortion_model = "plumb_bob";
    info0_msg.binning_x = 1;
    info0_msg.binning_y = 1;

    // read calibration information
    std::string cam_calib_filename = kitti_dir + "/calib_cam_to_cam.txt";
    std::ifstream cam_calib_file(cam_calib_filename);

    if (!cam_calib_file) {
        ROS_ERROR_STREAM("[KITTI Data] Unable to open file "
                         << cam_calib_filename);
        exit(1);
    }

    std::string ln;
    std::string token;

    Eigen::Vector3d t0, t1;

    while (std::getline(cam_calib_file, ln)) {
        std::stringstream ss(ln);

        ss >> token;

        if (token.find("P_rect_02") == 0) {
            for (int i = 0; i < 12; ++i) {
                ss >> info0_msg.P[i];
            }
        } else if (token.find("P_rect_03") == 0) {
            for (int i = 0; i < 12; ++i) {
                ss >> info1_msg.P[i];
            }
        }
    }

    std::cout << "Camera 0 P = {";
    for (double x : info0_msg.P) {
        std::cout << x << ", ";
    }
    std::cout << "}" << std::endl;
    std::cout << "Camera 1 P = {";
    for (double k : info1_msg.P) {
        std::cout << k << ", ";
    }
    std::cout << "}" << std::endl;
    
    std::string cam0_fmt_str = kitti_dir + "/image_02/data/%010d.png";
    std::string cam1_fmt_str = kitti_dir + "/image_03/data/%010d.png";
    size_t cam_next = 0;


    // Find first index
    while (cam_ts[cam_next] * rate < t_start)
        cam_next++;

    double t_offset = ros::Time::now().toSec() - cam_ts[cam_next];
    ROS_INFO_STREAM("T OFFSET = " << t_offset);
    // std::cout << "cam_ts size: " << cam_ts.size() << std::endl;

    while (ros::ok() && cam_next < cam_ts.size()-1) {

        if (cam_ts[cam_next] * rate > t_end) {
            break;
        }

        // read & publish cam
        char fname[1024];
        sprintf(fname, cam0_fmt_str.c_str(), cam_next);
        // std::cout << "read image" << cam_next << std::endl;
        // std::cout << fname << std::endl;
        cv::Mat img0 = cv::imread(fname);

        sprintf(fname, cam1_fmt_str.c_str(), cam_next);
        cv::Mat img1 = cv::imread(fname);

        cv_bridge::CvImage img0_msg;
        img0_msg.header.stamp = ros::Time(cam_ts[cam_next] * rate + t_offset);
        img0_msg.image = img0;
        img0_msg.encoding = sensor_msgs::image_encodings::BGR8;

        cv_bridge::CvImage img1_msg;
        img1_msg.header.stamp = ros::Time(cam_ts[cam_next] * rate + t_offset);
        img1_msg.image = img1;
        img1_msg.encoding = sensor_msgs::image_encodings::BGR8;

        double delay = cam_ts[cam_next] + t_offset - ros::Time::now().toSec();
        if (cam_next > 0 && delay > 0) {
            // ROS_INFO_STREAM("Sleeping for " << delay << " seconds.");
            ros::Duration(cam_ts[cam_next] + t_offset -
                          ros::Time::now().toSec())
              .sleep();
        }

        // ROS_INFO_STREAM("[Kitti data node] publishing t = " <<
        // cam_ts[cam_next]); ROS_INFO_STREAM("Data = " << data);

        cam0_pub.publish(img0_msg.toImageMsg());
        cam1_pub.publish(img1_msg.toImageMsg());

        // car_det_pub.publish(car_det_msg);
        // window_det_pub.publish(window_det_msg);

        info0_msg.header.stamp = ros::Time(cam_ts[cam_next] * rate + t_offset);
        cam0_info_pub.publish(info0_msg);

        info1_msg.header.stamp = ros::Time(cam_ts[cam_next] * rate + t_offset);
        cam1_info_pub.publish(info1_msg);

        // last_t = cam_ts[cam_next];
        cam_next++;
    }

    std::cout << "saving time offset..." << std::endl;
    std::string save_root = "/home/hanwen/code/object_slam/semanticSLAM/estim_result/";
    int seq_id;
    data_nh.getParam("data_seq_id", seq_id);
    // std::cout << "seq_id: " << typeid(seq_id).name() << seq_id << std::endl;
    // std::string t_offset_file_dir = "/home/hanwen/code/semanticSLAM/t_offset.txt";
    std::string t_offset_file_dir = save_root + std::string(3 - std::to_string(seq_id).length(), '0') + std::to_string(seq_id) + "/t_offset.txt";
    std::cout << "t_offset_file_dir: " << t_offset_file_dir << std::endl;
    // std::cout << "data_dir: " << kitti_dir << std::endl;
    std::ofstream t_offset_file(t_offset_file_dir);
    if (t_offset_file.is_open()){
        t_offset_file << std::setiosflags(std::ios::fixed)<<std::setprecision(10) << t_offset;
    }
    t_offset_file.close();
}