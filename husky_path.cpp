#include <ros/ros.h>
#include "ncrl_ugv_ctrl/husky_path.h"
#include "ncrl_ugv_ctrl/qptrajectory.h"
#include "ncrl_ugv_ctrl/ncrl_tf.h"
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <thread>
#include "std_msgs/Int32.h"
//declare global variable
ros::Publisher pub_cmd , path_pub ,lio_pub_path;
ros::Publisher husky_state ;
std::string  POS_topic;
double freq;

//control
struct PID pid_x;
struct PID pid_y;
double minCmdX, maxCmdX, minCmdW, maxCmdW;

//qp part
//goal_path show path which qp has solved
nav_msgs::Path goal_path;
geometry_msgs::Point vel;
geometry_msgs::Point acc;

//goal_pose 以world為座標
//lio_pose 以body frame為座標
ncrl_tf::Trans goal_pose, world2body, lio_pose;
tf::Transform world;

//lio
//lio_path show trajectory on body frame
nav_msgs::Path lio_path;

//global variable
int i = 0;
int count_;
int ENABLE_FEEDFORWARD = 0;
bool systemInit = false;
double init_time;

int c = -1;
void callback(const std_msgs::Int32 data){
	c = data.data;
}


//readparameter from launch parameter
bool readParameter(ros::NodeHandle &nh)
{
    bool result = true;
    // get topic name
    if (  !nh.getParam("POS_topic", POS_topic)){
        ROS_ERROR("Failed to get param 'Path_topic'");
        ROS_ERROR("Failed to get param 'POS_topic'");
        result = false;
    }

    // get pid variable
    if (!nh.getParam("X_KP", pid_x.KP) || !nh.getParam("X_KI", pid_x.KI) || !nh.getParam("X_KD", pid_x.KD)){
        ROS_ERROR("Failed to get param x axis PID");
        result = false;
    }
    if (!nh.getParam("Y_KP", pid_y.KP) || !nh.getParam("Y_KI", pid_y.KI) || !nh.getParam("Y_KD", pid_y.KD)){
        ROS_ERROR("Failed to get param y axis PID");
        result = false;
    }

    // get frequency
    if (!nh.getParam("FREQ", freq)){
        ROS_ERROR("Failed to get param 'FREQ'");
        result = false;
    }

    if (!nh.getParam("max_vel_x", maxCmdX) || !nh.getParam("min_vel_x", minCmdX) ||
        !nh.getParam("max_vel_w", maxCmdW) || !nh.getParam("min_vel_w", minCmdW)){
        ROS_ERROR("Failed to get confinement of cmd");
        result = false;
    }

    return result;
}

void cb_pos(const nav_msgs::Odometry::ConstPtr& msg){
    Eigen::Vector3d v_(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond q_(msg->pose.pose.orientation.w,
                          msg->pose.pose.orientation.x,
                          msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z);
    //lio_pose 有了現在收到的q 和 v translation
    ncrl_tf::setTrans(lio_pose, q_, v_);
}


void process()
{
    //ncrl::tf
    static tf::TransformBroadcaster br;
    ros::Rate r(freq);

    //要傳給husky的vel_cmd指令
    geometry_msgs::Twist vel_cmd;


    int flag = 1;
    while(ros::ok())
    {
	   while(c==-1)
	   {
	        c = getch();
	   }
        
        if (c != EOF)
        {
          switch(c)
          {
            case 49:     // key 1
              flag = 1;
            break;
            case 2:     // key 2
              flag = 2;
            break;
            case 4:     // key 3
              flag = 4;
            break;
            case 52:     // key 4
              ENABLE_FEEDFORWARD = 1 - ENABLE_FEEDFORWARD;
            break;
            case 119:    // key w
              vel_cmd.linear.x += 0.1;
            break;
            case 115:    // key s
              vel_cmd.linear.x += -0.1;
            break;
            case 100:    // key d
              vel_cmd.angular.z += -0.1;
            break;
            case 97:    // key a
              vel_cmd.angular.z += 0.1;
            break;
            case 114:    // key r
              vel_cmd.linear.x = 0;
              vel_cmd.angular.z = 0;
            break;
            case 105:    // key i
            goal_pose.v(0) += 0.1;
            break;
            case 106:    // key j
            goal_pose.v(1) += 0.1;
            break;
            case 107:    // key k
            goal_pose.v(0) -= 0.1;
            break;
            case 108:    // key l
            goal_pose.v(1) -= 0.1;
            break;
          }
        }
        if (flag == 1)
        {
            ROS_INFO(" ===== KEYBOARD CONTROL ===== ");
            ENABLE_FEEDFORWARD = 0;
            goal_pose.v = lio_pose.v;
            goal_pose.q = lio_pose.q;
        }
        else if (flag == 2||flag ==4)
        {
            ROS_INFO(" ===== TRACE ===== ");
        }

        if (ENABLE_FEEDFORWARD)
            ROS_INFO(" ===== ENABLE FEEDFORWARD TERM =====");

        if (flag == 1)
            pub_cmd.publish(vel_cmd);

        // choose suitable point
        if (flag == 2 || flag == 4)
        {
            ROS_INFO(" ===== enter ===== ");
            double max;
            double sample=0.02;
            qptrajectory plan;
            path_def path;
            trajectory_profile p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12;
            std::vector<trajectory_profile> data;
	    if (flag == 2){
		    p1.pos << 0.0,0,0;
		    p1.vel << 0.0,0.0,0;
		    p1.acc << 0.00,-0.0,0;
		    p1.yaw = 0;

		    p2.pos<< 1.5,0.0,0;
		    p2.vel<< 0,0,0;
		    p2.acc<< 0,0,0;
		    p2.yaw = 0;

		    p3.pos<< 2.5,1.5,0.0;
		    p3.vel<< 0,0,0;
		    p3.acc<< 0,0,0;
		    p3.yaw = 0;

		    p4.pos << 1.5,2.5,0;
		    p4.vel << 0,0,0;
		    p4.acc << 0,0,0;
		    p4.yaw = 0;

		    p5.pos << 0.0,3.5,0;
		    p5.vel << 0.0,0.0,0;
		    p5.acc << 0.00,-0.0,0;
		    p5.yaw = 0;

		    p6.pos << -1.0,5.0,0;
		    p6.vel << 0.0,0.0,0;
		    p6.acc << 0.00,-0.0,0;
		    p6.yaw = 0;

		    p7.pos << 1.0,6.5,0;
		    p7.vel << 0.0,0.0,0;
		    p7.acc << 0.00,-0.0,0;
		    p7.yaw = 0;

		    p8.pos << 4.0,6.5,0;
		    p8.vel << 0.0,0.0,0;
		    p8.acc << 0.00,-0.0,0;
		    p8.yaw = 0;



		    path.push_back(segments(p1,p2,5));
		    path.push_back(segments(p2,p3,4));
		    path.push_back(segments(p3,p4,2));
		    path.push_back(segments(p4,p5,2));
		    path.push_back(segments(p5,p6,2));
		    path.push_back(segments(p6,p7,2));
		    path.push_back(segments(p7,p8,4));
	    }
	    if (flag == 4){

		    p9.pos << 4.0,6.5,0.0;
		    p9.vel << 0.0,0.0,0;
		    p9.acc << 0.00,-0.0,0;
		    p9.yaw = -45;

		    p10.pos<< 5.0,5.0,0;
		    p10.vel<< 0,0,0;
		    p10.acc<< 0,0,0;
		    p10.yaw = 0;

		    p11.pos<< 4.0,0.0,0.0;
		    p11.vel<< 0,0,0;
		    p11.acc<< 0,0,0;
		    p11.yaw = 0;
        
		 
		    path.push_back(segments(p9,p10,2));
		    path.push_back(segments(p10,p11,3));
	    }
             

            data = plan.get_profile(path ,path.size(),sample);
            max = data.size();

            std::cout<<"ros::ok-------------"<<ros::ok<<std::endl;
            std::cout<<"max"<<max<<std::endl;

            while(ros::ok())
            {
                if(count_ >=max)
                {
                    world2body.v << 0,0,0;
                    Eigen::Vector3d euler(0,0,0);
                    world2body.q = ncrl_tf::Euler2Q(euler);

                    vel.x = 0;
                    vel.y = 0;
                    vel.z = 0;

                    acc.x = 0;
                    acc.y = 0;
                    acc.z = 0;
                    count_ =0;

                    vel_cmd.linear.x = 0;
                    vel_cmd.angular.z = 0;

                    std::cout<<"break---------------------------"<<std::endl;
                    //???
                    pub_cmd.publish(vel_cmd);
                    flag = 1;

		            std_msgs::Int32 fuck_you;
                    fuck_you.data = c;
                    husky_state.publish(fuck_you);
                    c=-1;
                    break;
                }

                else
                {

                std::cout<<"done1"<<std::endl;

                ros::Time current_time;
                current_time = ros::Time::now();

                goal_path.header.stamp=current_time;
                goal_path.header.frame_id="WORLD";

                geometry_msgs::PoseStamped this_pose_stamped;

                this_pose_stamped.header.stamp=current_time;
                this_pose_stamped.header.frame_id="WORLD";

                this_pose_stamped.pose.position.x = data[count_].pos[0];
                this_pose_stamped.pose.position.y = data[count_].pos[1];

                goal_path.poses.push_back(this_pose_stamped);

                world2body.v = world2body.q.inverse()* lio_pose.v + world2body.v;
                world2body.q = world2body.q * lio_pose.q;

                ncrl_tf::setTfTrans(world , world2body.q ,world2body.v);
                br.sendTransform(tf::StampedTransform(world, current_time ,"WORLD","now"));

                goal_pose.v << data[count_].pos[0] ,data[count_].pos[1] , data[count_].pos[2];

                vel.x = data[count_].vel[0];
                vel.y = data[count_].vel[1];
                vel.z = data[count_].vel[2];

                acc.x = data[count_].acc[0];
                acc.y = data[count_].acc[1];
                acc.z = data[count_].acc[2];
                count_ += 1 ;

                //<lio_path>
                lio_path.header.stamp=current_time;
                lio_path.header.frame_id="WORLD";

                geometry_msgs::PoseStamped lio_pose_stamped;

                lio_pose_stamped.header.stamp=current_time;
                lio_pose_stamped.header.frame_id="WORLD";

                lio_pose_stamped.pose.position.x = world2body.v(0);
                lio_pose_stamped.pose.position.y = world2body.v(1);

                lio_path.poses.push_back(lio_pose_stamped);

                //</lio_path>

                path_pub.publish(goal_path);
                lio_pub_path.publish(lio_path);


                }

                Eigen::Vector3d error,error_last;


                for(int i=0; i<max;i++)
                {
                    if(i ==0)
                    {
                        world2body.v << 0, 0, 0;
                    }

                    else
                    {

                        goal_pose.v = world2body.q * goal_pose.v + world2body.v;

                        error= goal_pose.v - lio_pose.v;
                        error_last = error;

                        float cmd_x, cmd_y;
                        pid_compute(pid_x, cmd_x, error(0), error_last(0), 0.001);
                        pid_compute(pid_y, cmd_y, error(1), error_last(1), 0.001);

                        std::cout <<"error_x" <<error <<std::endl;

                        Eigen::Vector3d cmd(cmd_x, cmd_y, 0);

                        //trans imu_init frame cmd into body frame
                        cmd = lio_pose.q.inverse() * cmd;

                        vel_cmd.linear.x = cmd(0);
                        vel_cmd.angular.z = cmd(1);

                        if (fabs(vel_cmd.linear.x) > maxCmdX){
                            vel_cmd.linear.x = vel_cmd.linear.x * maxCmdX / fabs(vel_cmd.linear.x);
                        } else if (fabs(vel_cmd.linear.x) < minCmdX){
                            vel_cmd.linear.x = 0;
                        }

                        //std::cout <<"vel_cmd.linear.x" <<vel_cmd.linear.x <<std::endl;

                        if (fabs(vel_cmd.angular.z) > maxCmdW){
                            vel_cmd.angular.z = vel_cmd.angular.z * maxCmdW / fabs(vel_cmd.angular.z);
                        } else if (fabs(vel_cmd.angular.z) < minCmdW){
                          vel_cmd.angular.z = 0;
                        }

                        pub_cmd.publish(vel_cmd);
                    }
                }
                r.sleep();
            }
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "husky_control");
    ros::NodeHandle nh;
    ros::Subscriber  husky_command = nh.subscribe ("husky/command", 20, callback); //Subscribe husky command
    bool init = readParameter(nh);

    if(init)
    {
        std::cout << "Trajectory generator"<<std::endl;

        std::cout << "\nHUSKY_PATH NODE : " <<
                     "\nPOS FEEDBACK TOPIC IS " << POS_topic <<  std::endl;

        std::cout << "\nPID CONTROL  : " <<
                     "\nX AXIS KP " << pid_x.KP << " KI " << pid_x.KI << " KD " << pid_x.KD <<
                     "\nY AXIS KP " << pid_y.KP << " KI " << pid_y.KI << " KD " << pid_y.KD << std::endl;

    }
    else
    {
        ros::shutdown();
    }

    ros::Subscriber sub_pos = nh.subscribe<nav_msgs::Odometry> (POS_topic, 20, cb_pos);
    husky_state = nh.advertise<std_msgs::Int32>("husky/finish",100);
    //qp的路徑
    path_pub = nh.advertise<nav_msgs::Path>("trajectory",1, true);
    pub_cmd = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 100);

    //lio的路徑
    lio_pub_path = nh.advertise<nav_msgs::Path> ("/future_path", 20);

    //init setting
    ncrl_tf::setTransFrame(lio_pose, "WORLD", "IMU");

    //process have no ()
    std::thread ctrl_process{process};

    std::cout<<"rrrrrrrrrrrrrrrrrrr"<<std::endl;
    ros::spin();
    return 0;
}
