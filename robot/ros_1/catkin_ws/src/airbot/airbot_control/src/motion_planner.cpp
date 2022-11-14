#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

double search_length, search_width_min, search_width_max, search_height, num_filter;
double wall_search_length, wall_search_width, wall_search_height, wall_num_filter;
double ground_clearance;

double fc_sl_range[2], fc_sw_range_right[2], fc_sw_range_left[2], fc_sh_range[2];
double fc_wsl_range[2], fc_wsw_range_right[2], fc_wsw_range_left[2], fc_wsh_range[2];
double rc_sl_range[2], rc_sw_range_right[2], rc_sw_range_left[2], rc_sh_range[2];
double rc_wsl_range[2], rc_wsw_range_right[2], rc_wsw_range_left[2], rc_wsh_range[2];
double fc_tf_x, fc_tf_y, fc_tf_z, rc_tf_x, rc_tf_y, rc_tf_z;

int motion_state;
double gain;
double linear_velocity;
geometry_msgs::Twist velocity;

double pub_rate;

void front_pc2_cb(sensor_msgs::PointCloud2ConstPtr const& front_pc2_data)
{
    if(motion_state == 1)
    {
        int count, num_r, num_l;
        double correction_factor;
        ros::Time st = ros::Time::now();
        sensor_msgs::PointCloud2ConstIterator<float> front_pc2_iter(*front_pc2_data, "x");
        double dist_r = fc_sl_range[1];
        double dist_l = fc_sl_range[1];
        count = 0;
        num_r = 0;
        num_l = 0;
        for (front_pc2_iter; front_pc2_iter != front_pc2_iter.end(); ++front_pc2_iter)
        {
            if(fc_sl_range[0] <= front_pc2_iter[2] && front_pc2_iter[2] <= fc_sl_range[1])
            {
                if(fc_sh_range[0] <= front_pc2_iter[1] && front_pc2_iter[1] <= fc_sh_range[1])
                {
                    if(fc_sw_range_right[0] <= front_pc2_iter[0] && front_pc2_iter[0] <= fc_sw_range_right[1])
                    {
                        num_r++;
                        if(front_pc2_iter[2] <= dist_r)
                        {
                            dist_r = front_pc2_iter[2];
                        }
                    }
                    else if(fc_sw_range_left[0] <= front_pc2_iter[0] && front_pc2_iter[0] <= fc_sw_range_left[1])
                    {
                        num_l++;
                        if(front_pc2_iter[2] <= dist_l)
                        {
                            dist_l = front_pc2_iter[2];
                        }
                    }
                }
            }
            count++;
        }
        dist_r = (num_r >= num_filter) ? dist_r : fc_sl_range[1];
        dist_l = (num_l >= num_filter) ? dist_l : fc_sl_range[1];
        correction_factor = (dist_l - dist_r)/(dist_l + dist_r);
        ROS_INFO_STREAM("Obstacle Dist Right: " << dist_r << " Obstacle Dist Left: " << dist_l << " Num Points: " << count);

        if (correction_factor == 0.0)
        {
            sensor_msgs::PointCloud2ConstIterator<float> front_pc2_iter(*front_pc2_data, "x");
            double width_r = fc_wsw_range_right[1];
            double width_l = fc_wsw_range_left[0];
            count = 0;
            num_r = 0;
            num_l = 0;
            for (front_pc2_iter; front_pc2_iter != front_pc2_iter.end(); ++front_pc2_iter)
            {
                if(fc_wsl_range[0] <= front_pc2_iter[2] && front_pc2_iter[2] <= fc_wsl_range[1])
                {
                    if(fc_wsh_range[0] <= front_pc2_iter[1] && front_pc2_iter[1] <= fc_wsh_range[1])
                    {
                        if(fc_wsw_range_right[0] <= front_pc2_iter[0] && front_pc2_iter[0] <= fc_wsw_range_right[1])
                        {
                            num_r++;
                            if(front_pc2_iter[0] <= width_r)
                            {
                                width_r = front_pc2_iter[0];
                            }
                        }
                        else if(fc_wsw_range_left[0] <= front_pc2_iter[0] && front_pc2_iter[0] <= fc_wsw_range_left[1])
                        {
                            num_l++;
                            if(width_l <= front_pc2_iter[0])
                            {
                                width_l = front_pc2_iter[0];
                            }
                        }
                    }
                }
                count++;
            }
            width_r = (num_r >= wall_num_filter) ? abs(width_r-fc_tf_x) : abs(fc_wsw_range_right[1]-fc_tf_x);
            width_l = (num_l >= wall_num_filter) ? abs(width_l-fc_tf_x) : abs(fc_wsw_range_left[0]-fc_tf_x);
            correction_factor = 2*(width_l - width_r)/(width_l + width_r);
            ROS_INFO_STREAM("Wall Dist Right: " << width_r << " Wall Dist Left: " << width_l << " Num Points: " << count);
        }
        velocity.linear.x = motion_state * linear_velocity;
        velocity.angular.z = gain * correction_factor;
        ros::Time et = ros::Time::now();
        ROS_INFO_STREAM("Correction Factor: " << correction_factor << " Loop Time: " << (et-st));
    }
}

void rear_pc2_cb(sensor_msgs::PointCloud2ConstPtr const& rear_pc2_data)
{
    if(motion_state == -1)
    {
        int count, num_r, num_l;
        double correction_factor;
        ros::Time st = ros::Time::now();
        sensor_msgs::PointCloud2ConstIterator<float> rear_pc2_iter(*rear_pc2_data, "x");
        double dist_r = fc_sl_range[1];
        double dist_l = fc_sl_range[1];
        count = 0;
        num_r = 0;
        num_l = 0;
        for (rear_pc2_iter; rear_pc2_iter != rear_pc2_iter.end(); ++rear_pc2_iter)
        {
            if(rc_sl_range[0] <= rear_pc2_iter[2] && rear_pc2_iter[2] <= rc_sl_range[1])
            {
                if(rc_sh_range[0] <= rear_pc2_iter[1] && rear_pc2_iter[1] <= rc_sh_range[1])
                {
                    if(rc_sw_range_right[0] <= rear_pc2_iter[0] && rear_pc2_iter[0] <= rc_sw_range_right[1])
                    {
                        num_r++;
                        if(rear_pc2_iter[2] <= dist_r)
                        {
                            dist_r = rear_pc2_iter[2];
                        }
                    }
                    else if(rc_sw_range_left[0] <= rear_pc2_iter[0] && rear_pc2_iter[0] <= rc_sw_range_left[1])
                    {
                        num_l++;
                        if(rear_pc2_iter[2] <= dist_l)
                        {
                            dist_l = rear_pc2_iter[2];
                        }
                    }
                }
            }
            count++;
        }
        dist_r = (num_r >= num_filter) ? dist_r : rc_sl_range[1];
        dist_l = (num_l >= num_filter) ? dist_l : rc_sl_range[1];
        correction_factor = (dist_l - dist_r)/(dist_l + dist_r);
        ROS_INFO_STREAM("Obstacle Dist Right: " << dist_r << " Obstacle Dist Left: " << dist_l << " Num Points: " << count);

        if (correction_factor == 0.0)
        {
            sensor_msgs::PointCloud2ConstIterator<float> rear_pc2_iter(*rear_pc2_data, "x");
            double width_r = rc_wsw_range_right[1];
            double width_l = rc_wsw_range_left[0];
            count = 0;
            num_r = 0;
            num_l = 0;
            for (rear_pc2_iter; rear_pc2_iter != rear_pc2_iter.end(); ++rear_pc2_iter)
            {
                if(rc_wsl_range[0] <= rear_pc2_iter[2] && rear_pc2_iter[2] <= rc_wsl_range[1])
                {
                    if(rc_wsh_range[0] <= rear_pc2_iter[1] && rear_pc2_iter[1] <= rc_wsh_range[1])
                    {
                        if(rc_wsw_range_right[0] <= rear_pc2_iter[0] && rear_pc2_iter[0] <= rc_wsw_range_right[1])
                        {
                            num_r++;
                            if(rear_pc2_iter[0] <= width_r)
                            {
                                width_r = rear_pc2_iter[0];
                            }
                        }
                        else if(rc_wsw_range_left[0] <= rear_pc2_iter[0] && rear_pc2_iter[0] <= rc_wsw_range_left[1])
                        {
                            num_l++;
                            if(width_l <= rear_pc2_iter[0])
                            {
                                width_l = rear_pc2_iter[0];
                            }
                        }
                    }
                }
                count++;
            }
            width_r = (num_r >= wall_num_filter) ? abs(width_r-rc_tf_x) : abs(rc_wsw_range_right[1]-rc_tf_x);
            width_l = (num_l >= wall_num_filter) ? abs(width_l-rc_tf_x) : abs(rc_wsw_range_left[0]-rc_tf_x);
            correction_factor = 2*(width_l - width_r)/(width_l + width_r);
            ROS_INFO_STREAM("Wall Dist Right: " << width_r << " Wall Dist Left: " << width_l << " Num Points: " << count);
        }

        velocity.linear.x = motion_state * linear_velocity;
        velocity.angular.z = gain * correction_factor;
        ros::Time et = ros::Time::now();
        ROS_INFO_STREAM("Correction Factor: " << correction_factor << " Loop Time: " << (et-st));
    }
}

void motion_state_cb(std_msgs::Int8 state_data) {
  motion_state = state_data.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airbot_motion_planner");
    ros::NodeHandle nh;
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    search_length = 1.0;
    search_width_min = 0.15;
    search_width_max = 0.23;
    search_height = 1.0;
    num_filter = 5;
    wall_search_length = 1.0;
    wall_search_width = 1.0;
    wall_search_height = 1.0;
    wall_num_filter = 3;
    ground_clearance = 0.05;
    linear_velocity = 0.08;
    gain = 0.1;
    pub_rate = 15.0;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::TransformStamped fc_bf_tf;
    geometry_msgs::TransformStamped rc_bf_tf;
    tf_buffer.canTransform("front_camera_depth_optical_frame", "base_footprint", ros::Time::now(), ros::Duration(60.0));
    tf_buffer.canTransform("rear_camera_depth_optical_frame", "base_footprint", ros::Time::now(), ros::Duration(60.0));
    fc_bf_tf = tf_buffer.lookupTransform("front_camera_depth_optical_frame", "base_footprint", ros::Time(0));
    rc_bf_tf = tf_buffer.lookupTransform("rear_camera_depth_optical_frame", "base_footprint", ros::Time(0));

    fc_tf_x = fc_bf_tf.transform.translation.x;
    fc_tf_y = fc_bf_tf.transform.translation.y;
    fc_tf_z = fc_bf_tf.transform.translation.z;

    rc_tf_x = rc_bf_tf.transform.translation.x;
    rc_tf_y = rc_bf_tf.transform.translation.y;
    rc_tf_z = rc_bf_tf.transform.translation.z;

    fc_sl_range[0] = 0.0;
    fc_sl_range[1] = search_length;
    fc_sw_range_right[0] = search_width_min+fc_tf_x;
    fc_sw_range_right[1] = search_width_max+fc_tf_x;
    fc_sw_range_left[0] = -search_width_max+fc_tf_x;
    fc_sw_range_left[1] = -search_width_min+fc_tf_x;
    fc_sh_range[0] = fc_tf_y-ground_clearance-search_height;
    fc_sh_range[1] = fc_tf_y-ground_clearance;
    fc_wsl_range[0] = 0.0;
    fc_wsl_range[1] = wall_search_length;
    fc_wsw_range_right[0] = search_width_min+fc_tf_x;
    fc_wsw_range_right[1] = wall_search_width+fc_tf_x;
    fc_wsw_range_left[0] = -wall_search_width+fc_tf_x;
    fc_wsw_range_left[1] = -search_width_min+fc_tf_x;
    fc_wsh_range[0] = fc_tf_y-ground_clearance-wall_search_height;
    fc_wsh_range[1] = fc_tf_y-ground_clearance;

    rc_sl_range[0] = 0.0;
    rc_sl_range[1] = search_length;
    rc_sw_range_right[0] = search_width_min+rc_tf_x;
    rc_sw_range_right[1] = search_width_max+rc_tf_x;
    rc_sw_range_left[0] = -search_width_max+rc_tf_x;
    rc_sw_range_left[1] = -search_width_min+rc_tf_x;
    rc_sh_range[0] = rc_tf_y-ground_clearance-search_height;
    rc_sh_range[1] = rc_tf_y-ground_clearance;
    rc_wsl_range[0] = 0.0;
    rc_wsl_range[1] = wall_search_length;
    rc_wsw_range_right[0] = search_width_min+rc_tf_x;
    rc_wsw_range_right[1] = wall_search_width+rc_tf_x;
    rc_wsw_range_left[0] = -wall_search_width+rc_tf_x;
    rc_wsw_range_left[1] = -search_width_min+rc_tf_x;
    rc_wsh_range[0] = rc_tf_y-ground_clearance-wall_search_height;
    rc_wsh_range[1] = rc_tf_y-ground_clearance;

    ros::Subscriber front_pc2_sub = nh.subscribe("/front_camera/depth/color/points", 1, front_pc2_cb);
    ros::Subscriber rear_pc2_sub = nh.subscribe("/rear_camera/depth/color/points", 1, rear_pc2_cb);
    ros::Subscriber motion_state_sub = nh.subscribe("/airbot_motion_state", 1, motion_state_cb);

    ros::Rate loop_rate(pub_rate);

    while (ros::ok())
    {
        if(motion_state == 0)
        {
            velocity.linear.x = 0.0;
            velocity.angular.z = 0.0;
        }
        if(motion_state != 10)
        {
            cmd_pub.publish(velocity);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
