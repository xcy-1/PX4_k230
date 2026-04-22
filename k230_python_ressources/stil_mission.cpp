#include <ros/ros.h>
#include <iostream>
#include <vector>

// MAVROS 消息头文件
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std;
enum MissionState {
    WAITING,
    TAKEOFF,
    SEARCHING,
    TRACKING, // 视觉伺服对准
    RESCUE,   // 投放/救援
    RETURN
};

class ReconMission {
public:
    ReconMission() : mission_state_(WAITING) //构造函数执行相关的初始化的操作
    {
        state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &ReconMission::state_cb, this);
        local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &ReconMission::pos_cb, this);
        vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
        pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        img_center_x_ = 320;
        img_center_y_ = 240;
    }

    void run() {
        ros::Rate rate(20.0); // 20Hz 控制频率
        while (ros::ok()) 
        {
            switch (mission_state_)
            {
                case WAITING:
                    if (current_state_.connected && current_state_.mode != "OFFBOARD")
                    {
                        // 这里为了演示，自动切 OFFBOARD。实际比赛建议遥控器切开关触发。
                         offboard_and_arm();
                    } 
                    else if (current_state_.mode == "OFFBOARD" && current_state_.armed) 
                    {
                        mission_state_ = TAKEOFF;//执行起飞的程序
                    }
                    break;

                case TAKEOFF:
                    takeoff_logic();
                    break;

                case SEARCHING:
                    //search_logic();
                    break;

                case TRACKING:
                    // tracking_logic();
                    break;
                
                case RESCUE:
                    ROS_INFO("sucess everything test");
                    // TODO: 添加控制舵机/GPIO的代码
                    ros::Duration(2.0).sleep(); // 模拟投送耗时
                    mission_state_ = RETURN; // 或者是继续 SEARCHING
                    break;
                
                case RETURN:
                    // 简单的降落逻辑
                    land_logic();
                    break;
            }

            ros::spinOnce();
            rate.sleep();
        }
    }
private:

    ros::NodeHandle nh_;
    ros::Subscriber state_sub_, local_pos_sub_;
    ros::Publisher vel_pub_, pos_pub_;
    ros::ServiceClient arming_client_, set_mode_client_;

    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    MissionState mission_state_;
    bool has_target_ = false;
    int16_t target_cx_ = 0, target_cy_ = 0;
    uint8_t target_class_ = 0;
    int img_center_x_, img_center_y_;
    const float TAKEOFF_ALT = 2.0; // 起飞高度 2米
    void state_cb(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }
    void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
    }    
    void offboard_and_arm() {
        // 设置起飞点
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2;
        pos_pub_.publish(pose);
        for (int i = 100; ros::ok() && i > 0; --i)
        {
            pose.header.stamp = ros::Time::now();
            pos_pub_.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))) 
        {
            // 调用服务客户端，发送切换到OFFBOARD模式的请求
            // 同时判断服务调用是否成功 且 飞控是否已接收模式切换指令
            if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) 
            {
                ROS_INFO("Offboard enabled"); // 打印日志：Offboard模式已启用
            }
            last_request_ = ros::Time::now(); // 更新上次请求时间为当前时间
        }
        // 2. 外层else：当前已经是OFFBOARD模式 或 距离上次请求不足5秒
        else 
        {
            // 内层判断：无人机未解锁 且 距离上次请求时间超过5秒
            if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))) 
            {
                arming_client_.call(arm_cmd);//解锁请求
                if (arming_client_.call(arm_cmd) && arm_cmd.response.success) 
                {
                    ROS_INFO("Vehicle armed"); // 打印日志：无人机已解锁
                }
                last_request_ = ros::Time::now(); // 更新上次请求时间为当前时间
            }
        }
    }

    void takeoff_logic() {
        // 发布位置设定点
        static int8_t statu=0;
        geometry_msgs::PoseStamped pose;


        switch (statu)
        {
            case 0:
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 2;
            pos_pub_.publish(pose);
            if(current_pose_.pose.position.z>=2-0.1){statu = 1;}
                break;
            case 1:
            pose.pose.position.x = 1;
            pose.pose.position.y = 0;
            pose.pose.position.z = 2;
            if(current_pose_.pose.position.x>=1-0.1)                
                statu = 2;
                break;
            case 2:
            pose.pose.position.x = 1;
            pose.pose.position.y = -1;
            pose.pose.position.z = 2;
            if(current_pose_.pose.position.y<=-1+0.1)
                statu = 3;
                break;
            case 3:
            pose.pose.position.x = 0;
            pose.pose.position.y = -1;
            pose.pose.position.z = 2;
            if(current_pose_.pose.position.x<=0+0.1)
                statu = 4;
                break;
            case 4:
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 2;
            if(current_pose_.pose.position.y>=0-0.1)
                {
                    mission_state_ = SEARCHING;
                    statu = 0;
                }
                break;
            default:
                break;
        }

    }

    void search_logic() {
        // 简单策略：向前慢慢飞，或者悬停等待
        // 也可以在这里写航点逻辑
        geometry_msgs::TwistStamped vel_msg;
        vel_msg.twist.linear.x = 0.5; // 向前 0.5m/s
        vel_msg.twist.linear.y = 0.0;
        vel_msg.twist.linear.z = 0.0;
        vel_pub_.publish(vel_msg);

        // 如果发现目标
        if (has_target_) {
            ROS_INFO("Target Found! Switching to Tracking.");
            mission_state_ = TRACKING;
        }
    }


    void tracking_logic() 
    {
        if (!has_target_) 
        {
            // 丢失目标，切回搜索模式
            mission_state_ = SEARCHING;
            return;
        }

        // PID 参数 (需要根据实际情况调试)
        float Kp_x = 0.005; // 像素转速度的比例系数
        float Kp_y = 0.005;

        // 计算误差 (图像中心 - 目标中心)
        int error_x = img_center_x_ - target_cx_; // 图像水平误差
        int error_y = img_center_y_ - target_cy_; // 图像垂直误差

        // 坐标系转换：
        // 摄像头下视时：
        // 图像上方(Y减小) 对应 机头前方 (Body X)
        // 图像右方(X增加) 对应 机身右方 (Body Y)
        
        geometry_msgs::TwistStamped vel_cmd;
        
        // 图像Y误差控制飞机X轴速度（前后）
        vel_cmd.twist.linear.x = error_y * Kp_x; 
        
        // 图像X误差控制飞机Y轴速度（左右）
        // 注意方向：如果目标在图像右边(cx大)，error_x为负，飞机应该向右飞(Body Y负? 需确认机身坐标系)
        // MAVROS Body Frame: X前, Y左, Z上 (FLU)
        // 目标在右边 -> 飞机需要向右(Y负) -> error_x(负) * Kp -> 负值. 逻辑正确.
        vel_cmd.twist.linear.y = error_x * Kp_y; 
        
        vel_cmd.twist.linear.z = 0.0; // 保持高度

        // 限速 (安全第一)
        if (vel_cmd.twist.linear.x > 0.5) vel_cmd.twist.linear.x = 0.5;
        if (vel_cmd.twist.linear.x < -0.5) vel_cmd.twist.linear.x = -0.5;
        if (vel_cmd.twist.linear.y > 0.5) vel_cmd.twist.linear.y = 0.5;
        if (vel_cmd.twist.linear.y < -0.5) vel_cmd.twist.linear.y = -0.5;

        vel_pub_.publish(vel_cmd);

        // 如果误差足够小，且稳定
        if (abs(error_x) < 20 && abs(error_y) < 20) {
            static int stable_count = 0;
            stable_count++;
            if (stable_count > 40) { // 连续2秒稳定 (20Hz * 2s)
                mission_state_ = RESCUE;
                stable_count = 0;
            }
        } else {
             stable_count = 0;
        }
    }
    
    void land_logic() {
        // 发送 AUTO.LAND 模式
        mavros_msgs::SetMode land_set_mode;
        land_set_mode.request.custom_mode = "AUTO.LAND";
        set_mode_client_.call(land_set_mode);
    }

    ros::Time last_request_;
    int stable_count = 0;    
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "cuadc_recon_node");
    ReconMission mission;
    mission.run();
    return 0;
}

