#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/CameraInfo.h>

#include <boost/format.hpp>

int main(int argc, char* argv[])
{

    // init node
    ros::init(argc, argv, "republish_usad2_node");
    ros::NodeHandle nh;

    rosbag::Bag bag;
    //bag.open("/media/ballardini/storage/datasets/usad2/onedrive-via_innovazione2.bag", rosbag::bagmode::Read);
    //bag.open("/media/ballardini/TOSHIBA EXT/saved/A4-4.bag", rosbag::bagmode::Read);
    bag.open("/media/ballardini/TOSHIBA EXT/splitted/A4-4_0.bag", rosbag::bagmode::Read);

    ROS_INFO_STREAM("open");

    std::vector<std::string> topics;

    rosbag::View view_left(bag, rosbag::TopicQuery(std::string("/stereo/left/image_raw")));
    rosbag::View view_right(bag, rosbag::TopicQuery(std::string("/stereo/right/image_raw")));

    rosbag::View::iterator left_iterator = view_left.begin();
    rosbag::View::iterator right_iterator = view_right.begin();

    sensor_msgs::Image::Ptr img_l,img_r;

    rosbag::MessageInstance &l_it = *left_iterator;
    rosbag::MessageInstance &r_it = *right_iterator;
    img_l = l_it.instantiate<sensor_msgs::Image>();
    img_r = r_it.instantiate<sensor_msgs::Image>();


    image_transport::ImageTransport it(nh);
    image_transport::Publisher p_r = it.advertise("/stereo/right/image_raw", 1);
    image_transport::Publisher p_l = it.advertise("/stereo/left/image_raw", 1);

    ros::Publisher p_cil = nh.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info", 1);
    ros::Publisher p_cir = nh.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info", 1);

    sensor_msgs::CameraInfo cil,cir;

    ////////////////////////////////////////////////////////////////////////////
    //RIGHT CAMERA
    cir.width = 1312;
    cir.height  = 541;
    cir.distortion_model = "plumb_bob";
    cir.D = {-0.239563, 0.084455, -0.002111, -0.000161, 0.000000};
    cir.K = {851.257609, 0.000000, 637.135422, 0.000000, 851.168364, 154.698218, 0.000000, 0.000000, 1.000000};
    cir.R = {0.999997, -0.001336, 0.002140, 0.001354, 0.999963, -0.008449, -0.002129, 0.008452, 0.999962};
    cir.P = {834.752703, 0.000000, 612.806038, -190.372340, 0.000000, 834.752703, 158.305822, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000};
    cir.roi.do_rectify = false;
    cir.header.frame_id = "stereo_rig";

    //LEFT CAMERA
    cil.width = 1312;
    cil.height  = 541;
    cil.distortion_model = "plumb_bob";
    cil.D = {-0.251963, 0.096396, -0.002254, -0.001636, 0.000000};
    cil.K = {853.415831, 0.000000, 641.228450, 0.000000, 852.608357, 167.569232, 0.000000, 0.000000, 1.000000};
    cil.R = {0.999479, -0.000960, 0.032258, 0.000688, 0.999964, 0.008463, -0.032265, -0.008436, 0.999444};
    cil.P = {834.752703, 0.000000, 612.806038, 0.000000, 0.000000, 834.752703, 158.305822, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000};
    cil.roi.do_rectify = false;
    cil.header.frame_id = "stereo_rig";
    ////////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////////
    // CALIBRATION FROM calibrationdata-06-04-2017 set 2 and 3
    //RIGHT CAMERA
    cir.width = 1312;
    cir.height  = 541;
    cir.distortion_model = "plumb_bob";
    cir.D = {-0.227628, 0.074452, -0.000792, -0.000156, 0.000000};
    cir.K = {841.942265, 0.000000, 637.938525, 0.000000, 842.485046, 263.755081, 0.000000, 0.000000, 1.000000};
    cir.R = {0.999967, -0.001377, -0.007973, 0.001402, 0.999994, 0.003161, 0.007969, -0.003172, 0.999963};
    cir.P = {850.361689, 0.000000, 642.190849, -422.417646, 0.000000, 850.361689, 267.709890, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000};
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


    ros::Rate r(20);        // 10 hz
    int published_images=0; // Counter

    // SAVING IMAGE PART
    cv::Mat image;
    boost::format filename_format_;
    filename_format_.parse(std::string("frame%010i.jpg"));

    for(; ((left_iterator!=view_left.end()) && (right_iterator !=view_right.end())) ; )        
    {
        ros::Time time_l = img_l->header.stamp;
        ros::Time time_r = img_r->header.stamp;
        ros::Duration diff  = time_l - time_r;
        ros::Duration threshold = ros::Duration(0.05);

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


        rosbag::MessageInstance &l_image_pointer = *left_iterator;
        rosbag::MessageInstance &r_image_pointer = *right_iterator;
        img_l = l_image_pointer.instantiate<sensor_msgs::Image>();
        img_r = r_image_pointer.instantiate<sensor_msgs::Image>();
        ROS_DEBUG_STREAM("L: " << img_l->header.seq          << "\tR: " << img_r->header.seq);
        ROS_DEBUG_STREAM("L: " << img_l->header.stamp.sec    << "\tR: " << img_r->header.stamp.sec );
        ROS_DEBUG_STREAM("L: " << img_l->header.stamp.nsec   << "\tR: " << img_r->header.stamp.nsec );
        ROS_WARN_STREAM("Difference: " << diff);


        //        ros::Time t;
        //        t=ros::Time::now();
        //        img_r->header.stamp = t;
        //        img_l->header.stamp = t;
        //        cil.header.stamp = t;
        //        cir.header.stamp = t;

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

        image = cv_bridge::toCvShare(img_l, "bgr8")->image;
        std::string filename = (filename_format_ % published_images).str();
        cv::imwrite("left/"+filename,image);

        image = cv_bridge::toCvShare(img_r, "bgr8")->image;
        filename = (filename_format_ % published_images).str();
        cv::imwrite("right/"+filename,image);

        cv::namedWindow("image_left");
        cv::imshow("image_left",image);
        cv::waitKey(1);

        left_iterator++;
        right_iterator++;
        
        ROS_INFO_STREAM("Image: " << published_images++);
        r.sleep();
        ros::spinOnce();
    }

    ROS_INFO_STREAM("the end");

    bag.close();

}



// LEFT.yaml
//
// image_width: 1312
// image_height: 541
// camera_name: narrow_stereo/left
// camera_matrix:
//   rows: 3
//   cols: 3
//   data: [853.415831, 0.000000, 641.228450, 0.000000, 852.608357, 167.569232, 0.000000, 0.000000, 1.000000]
// distortion_model: plumb_bob
// distortion_coefficients:
//   rows: 1
//   cols: 5
//   data: [-0.251963, 0.096396, -0.002254, -0.001636, 0.000000]
// rectification_matrix:
//   rows: 3
//   cols: 3
//   data: [0.999479, -0.000960, 0.032258, 0.000688, 0.999964, 0.008463, -0.032265, -0.008436, 0.999444]
// projection_matrix:
//   rows: 3
//   cols: 4
//   data: [834.752703, 0.000000, 612.806038, 0.000000, 0.000000, 834.752703, 158.305822, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]








// RIGHT.yaml
//
// image_width: 1312
// image_height: 541
// camera_name: narrow_stereo/right
// camera_matrix:
//   rows: 3
//   cols: 3
//   data: [851.257609, 0.000000, 637.135422, 0.000000, 851.168364, 154.698218, 0.000000, 0.000000, 1.000000]
// distortion_model: plumb_bob
// distortion_coefficients:
//   rows: 1
//   cols: 5
//   data: [-0.239563, 0.084455, -0.002111, -0.000161, 0.000000]
// rectification_matrix:
//   rows: 3
//   cols: 3
//   data: [0.999997, -0.001336, 0.002140, 0.001354, 0.999963, -0.008449, -0.002129, 0.008452, 0.999962]
// projection_matrix:
//   rows: 3
//   cols: 4
//   data: [834.752703, 0.000000, 612.806038, -190.372340, 0.000000, 834.752703, 158.305822, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
