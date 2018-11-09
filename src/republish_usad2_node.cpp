


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/CameraInfo.h>

#include <boost/format.hpp>
#include <std_msgs/Bool.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <string>

bool go;
bool cambia;

//1st step
//std::string full_filename_to_read="/media/ballardini/TOSHIBA/saved/A4-5_3.bag";
//std::string full_filename_to_save="/media/ballardini/TOSHIBA/toreindex/A4-5_3.bag";

//2nd step
std::string full_filename_to_read="/home/cattaneod/dataset/merged.bag";

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

void sync_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    if (cambia)
        go=true;
}

int main(int argc, char* argv[])
{
    go=false;
    cambia = true;

    // init node
    ros::init(argc, argv, "republish_usad2_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/sync", 1, sync_Callback);

    ROS_INFO_STREAM("opening");


    rosbag::Bag bag;
    bag.open(full_filename_to_read, rosbag::bagmode::Read);

#ifdef ENABLE_SYNC_MODE
    ROS_INFO_STREAM("open OK! waiting for sync");
#else
    ROS_INFO_STREAM("open OK!");
#endif


    std::vector<std::string> topics;

    rosbag::View view_left(bag, rosbag::TopicQuery(std::string("/stereo/left/image_raw")));
    rosbag::View view_right(bag, rosbag::TopicQuery(std::string("/stereo/right/image_raw")));

    rosbag::View::iterator left_iterator = view_left.begin();
    rosbag::View::iterator right_iterator = view_right.begin();

    sensor_msgs::Image::Ptr img_l,img_r;

    //rosbag::MessageInstance &l_it = *left_iterator;
    //rosbag::MessageInstance &r_it = *right_iterator;
    //img_l = l_it.instantiate<sensor_msgs::Image>();
    //img_r = r_it.instantiate<sensor_msgs::Image>();

    image_transport::ImageTransport it(nh);
    image_transport::Publisher p_r = it.advertise("/stereo/right/image_raw", 1);
    image_transport::Publisher p_l = it.advertise("/stereo/left/image_raw", 1);

    ros::Publisher p_cil = nh.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info", 1);
    ros::Publisher p_cir = nh.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info", 1);

    sensor_msgs::CameraInfo cil,cir;

    ////////////////////////////////////////////////////////////////////////////
    // CALIBRATION FROM calibrationdata-06-04-2017 set 2 and 3
    //RIGHT CAMERA
    cir.width = 1312;
    cir.height  = 541;
    cir.distortion_model = "plumb_bob";
    cir.D = {-0.227628, 0.074452, -0.000792, -0.000156, 0.000000};
    cir.K = {841.942265, 0.000000, 637.938525, 0.000000, 842.485046, 263.755081, 0.000000, 0.000000, 1.000000};
    cir.R = {0.999967, -0.001377, -0.007973, 0.001402, 0.999994, 0.003161, 0.007969, -0.003172, 0.999963};
    cir.P = {850.361689, 0.000000, 642.190849, -422.417646,
             0.000000, 850.361689, 267.709890, 0.000000,
             0.000000, 0.000000, 1.000000, 0.000000};
    cir.roi.do_rectify = false;
    cir.header.frame_id = "stereo_rig";

    //LEFT CAMERA
    cil.width = 1312;
    cil.height  = 541;
    cil.distortion_model = "plumb_bob";
    cil.D = {-0.234140, 0.079746, -0.001740, -0.000558, 0.000000};
    cil.K = {841.823676, 0.000000, 649.285940, 0.000000, 841.651743, 274.552700, 0.000000, 0.000000, 1.000000};
    cil.R = {0.999984, 0.001497, 0.005484, -0.001480, 0.999994, -0.003171, -0.005489, 0.003163, 0.999980};
    cil.P = {850.361689, 0.000000, 642.190849, 0.000000, 0.000000, 850.361689, 267.709890, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000};
    cil.roi.do_rectify = false;
    cil.header.frame_id = "stereo_rig";
    ////////////////////////////////////////////////////////////////////////////


    ros::Rate r(10);        // 10 hz
    int published_images=0; // Counter

    // SAVING IMAGE PART
    cv::Mat image_right, image_left;
    boost::format filename_format_;
    filename_format_.parse(std::string("frame%010i.png"));

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

#ifdef SEND_PCD
    boost::format filename_format_pcd_;
    filename_format_pcd_.parse(std::string("frame%010i.pcd"));
    boost::shared_ptr<ros::Publisher> pc_pub_;
    pc_pub_.reset(new ros::Publisher(nh.advertise<PointCloud>("/elas_ros/point_cloud", 1)));
#endif

#ifdef RESAVE_BAG
    rosbag::Bag saving_bag;
    saving_bag.open(full_filename_to_save, rosbag::bagmode::Write);

#endif

    for(; ((left_iterator!=view_left.end()) && (right_iterator !=view_right.end())) ; )
    {

#ifdef ENABLE_SYNC_MODE
        while (!go)
        {
            ros::spinOnce();
        }
#endif
        ROS_INFO_STREAM("GO");
        cambia=false;

        rosbag::MessageInstance &l_image_pointer = *left_iterator;
        rosbag::MessageInstance &r_image_pointer = *right_iterator;
        img_l = l_image_pointer.instantiate<sensor_msgs::Image>();
        img_r = r_image_pointer.instantiate<sensor_msgs::Image>();

        ros::Time time_l = img_l->header.stamp;
        ros::Time time_r = img_r->header.stamp;
        ros::Duration diff  = time_l - time_r;
        //ros::Duration threshold = ros::Duration(0.05);
        ros::Duration threshold = ros::Duration(0.01); //100 KM/h / 3.6 > 27.7m/s * 0.005 ~ 0.14m

        while ((diff > threshold) || (diff < -threshold))
        {
            ROS_WARN_STREAM("MISSING PACKAGE! Difference" << diff);

            //            ROS_INFO_STREAM(img_l->header.seq << " " << img_r->header.seq);
            //            ROS_INFO_STREAM(img_l->header.stamp.sec << " " << img_r->header.stamp.sec );
            //            ROS_INFO_STREAM(img_l->header.stamp.nsec << " " << img_r->header.stamp.nsec << "\n");

            if (diff > threshold)
            {
                right_iterator++;
            }
            else
            {
                left_iterator++;
            }

            rosbag::MessageInstance &l_image_pointer = *left_iterator;
            rosbag::MessageInstance &r_image_pointer = *right_iterator;

            img_l = l_image_pointer.instantiate<sensor_msgs::Image>();
            img_r = r_image_pointer.instantiate<sensor_msgs::Image>();

            time_l = img_l->header.stamp;
            time_r = img_r->header.stamp;
            diff  = time_l - time_r;
            ros::spinOnce();
        }

        ROS_DEBUG_STREAM("L: " << img_l->header.seq          << "\tR: " << img_r->header.seq);
        ROS_DEBUG_STREAM("L: " << img_l->header.stamp.sec    << "\tR: " << img_r->header.stamp.sec );
        ROS_DEBUG_STREAM("L: " << img_l->header.stamp.nsec   << "\tR: " << img_r->header.stamp.nsec );
        ROS_WARN_STREAM("Difference: " << diff);

#ifdef SAVE_TIMESTAMPS
        std::ofstream timestamp_left;
        std::string timestamp_filename_left = "/home/ballardini/Desktop/republish_timestamp_left.txt";
        timestamp_left.open (timestamp_filename_left, std::ios::app);
        timestamp_left << img_l->header.seq << ";" << img_l->header.stamp << std::endl;
        timestamp_left.close();

        std::ofstream timestamp_right;
        std::string timestamp_filename_right = "/home/ballardini/Desktop/republish_timestamp_right.txt";
        timestamp_right.open (timestamp_filename_right, std::ios::app);
        timestamp_right << img_r->header.seq << ";" << img_r->header.stamp << std::endl;
        timestamp_right.close();
#endif

#ifdef SEND_PCD
        std::string PCD_folder = "/home/ballardini/Desktop/PCD/A4-5_1.bag/";
        std::string filename = PCD_folder+(filename_format_pcd_ % published_images).str();
        PointCloud::Ptr point_cloud(new PointCloud());
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *point_cloud) == -1) //* load the file
        {
            ROS_ERROR_STREAM ("Couldn't read file " << filename << " \n");
            return (-1);
        }
        ROS_INFO_STREAM ("Publishing PCD: " << filename);
        pc_pub_->publish(point_cloud);
#endif

#ifdef SAME_TIMESTAMP
        img_r->header.stamp = img_l->header.stamp;
#endif

        cil.header.stamp.sec    = img_l->header.stamp.sec;
        cil.header.stamp.nsec   = img_l->header.stamp.nsec;
        cir.header.stamp.sec    = img_r->header.stamp.sec;
        cir.header.stamp.nsec   = img_r->header.stamp.nsec;

        ROS_DEBUG_STREAM("ci-sec: " << cil.header.stamp.sec    << "\tR: " << cir.header.stamp.sec );
        ROS_DEBUG_STREAM("ci-nse: " << cil.header.stamp.nsec   << "\tR: " << cir.header.stamp.nsec );

        p_l.publish(img_l);
        p_r.publish(img_r);

        p_cir.publish(cir);
        p_cil.publish(cil);

        ROS_DEBUG_STREAM("ci-sec: " << cil.header.stamp.sec    << "\tR: " << cir.header.stamp.sec );
        ROS_DEBUG_STREAM("ci-nse: " << cil.header.stamp.nsec   << "\tR: " << cir.header.stamp.nsec );

        std::string filename = (filename_format_ % published_images).str();
        image_left = cv_bridge::toCvShare(img_l, "bgr8")->image;
        image_right = cv_bridge::toCvShare(img_r, "bgr8")->image;

#ifdef SAVE_UNRECTIFIED_IMAGES
        cv::imwrite("/home/ballardini/Desktop/UNRECT/left/"+filename,image_left,compression_params);
        cv::imwrite("/home/ballardini/Desktop/UNRECT/right/"+filename,image_right,compression_params);
#endif

#ifdef RESAVE_BAG
        saving_bag.write("/stereo/right/image_raw", img_r->header.stamp, *img_r);
        saving_bag.write("/stereo/left/image_raw" , img_r->header.stamp, *img_l);
        saving_bag.write("/stereo/right/camera_info", img_r->header.stamp, cir);
        saving_bag.write("/stereo/left/camera_info" , img_r->header.stamp, cil);
#endif

        //cv::namedWindow("image_left");
        //cv::imshow("image_left",image_left);
        //cv::waitKey(1);

        left_iterator++;
        right_iterator++;

        go=false;
        cambia=true;
        
        ROS_INFO_STREAM("Image: " << published_images++);

#ifndef ENABLE_SYNC_MODE
        //r.sleep();
#endif
        ros::spinOnce();
    }

    ROS_INFO_STREAM("the end");
#ifdef RESAVE_BAG
    saving_bag.close();
    ROS_INFO_STREAM("closing bagfile");
#endif

    bag.close();

}
