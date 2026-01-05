/**
 * @file no_vis_mixed_node.cpp
 * @brief 混合控制(位置+速度) + 相对坐标系导航 (无视觉纯净版)
 * 按照指定航点飞行，不依赖摄像头
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

// --- 全局配置变量 ---
// 起飞时自动记录，用于将 "向前/向左" 转换为 "ENU X/Y"
double initial_yaw = 0.0;       // 起飞时的机头朝向 (弧度)
double initial_pos_x = 0.0;     // 起飞时的ENU X坐标
double initial_pos_y = 0.0;     // 起飞时的ENU Y坐标
bool initial_recorded = false;  // 标记是否已记录

// --- 枚举定义 ---
enum ControlMode { POS_MODE, VEL_MODE };

struct Waypoint {
    double fwd; // 相对起飞机头的"向前"距离 (米)
    double lat; // 相对起飞机头的"向左"距离 (米, 负数为向右)
    double z;   // 高度 (米)
    ControlMode mode; // 使用位置控制 还是 速度PID控制
};

// --- 全局ROS变量 ---
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

// --- PID 类 ---
class PIDController {
public:
    double kp, ki, kd, limit;
    double error_last = 0, error_sum = 0;

    PIDController(double p, double i, double d, double lim) 
        : kp(p), ki(i), kd(d), limit(lim) {}

    double calculate(double target, double current, double dt) {
        if (dt < 0.000001) return 0;
        double error = target - current;
        error_sum += error * dt;
        // 积分限幅
        if (error_sum > limit/ki && ki != 0) error_sum = limit/ki;
        if (error_sum < -limit/ki && ki != 0) error_sum = -limit/ki;
        
        double derivative = (error - error_last) / dt;
        error_last = error;
        double output = kp * error + ki * error_sum + kd * derivative;
        return std::max(std::min(output, limit), -limit);
    }
    void reset() { error_last = 0; error_sum = 0; }
};

// --- 回调函数 ---
void state_cb(const mavros_msgs::State::ConstPtr& msg){ current_state = *msg; }
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){ current_pose = *msg; }

// 计算三维距离
double get_distance(double x, double y, double z, const geometry_msgs::Point& p) {
    return sqrt(pow(x - p.x, 2) + pow(y - p.y, 2) + pow(z - p.z, 2));
}

// === 核心坐标转换 (Body Frame -> World ENU Frame) ===
// 输入: 相对起飞方向的 fwd(前), lat(左)
// 输出: 全局坐标系下的 x, y
void transform_body_to_enu(double fwd, double lat, double &out_x, double &out_y) {
    if (!initial_recorded) {
        out_x = fwd; out_y = lat; return;
    }
    // 旋转矩阵公式: 将相对位移旋转 initial_yaw 度，并加上初始偏移
    out_x = initial_pos_x + fwd * cos(initial_yaw) - lat * sin(initial_yaw);
    out_y = initial_pos_y + fwd * sin(initial_yaw) + lat * cos(initial_yaw);
}

// ================= MAIN =================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "no_vis_mixed_node");
    ros::NodeHandle nh;

    // ROS 通信
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);
    while(ros::ok() && !current_state.connected){ ros::spinOnce(); rate.sleep(); }

    // === 定义航点 (按照你的代码逻辑) ===
    // 格式: {向前(m), 向左(m), 高度(m), 模式}
    std::vector<Waypoint> mission_waypoints;
    
    // 0. 起飞 (原地升空)
    mission_waypoints.push_back({0.0, 0.0, 2.8, POS_MODE}); 
    
    // 1. 飞向点1 (6.3, 2.0) - 位置模式
    mission_waypoints.push_back({6.3, 2.0, 2.8, POS_MODE});
    
    // 2. 飞向点2 (8.9, 2.0) - 速度模式测试 (原视觉段)
    // 即使没有视觉，我们也可以用 VEL_MODE 测试 PID 导航是否平滑
    mission_waypoints.push_back({8.9, 2.0, 2.8, VEL_MODE});
    
    // 3. 飞向点3 (8.9, -2.0) - 位置模式
    mission_waypoints.push_back({8.9, -2.0, 2.8, POS_MODE});
    
    // 4. 飞向点4 (6.3, -2.0) - 速度模式测试 (原视觉段)
    mission_waypoints.push_back({6.3, -2.0, 2.8, VEL_MODE});
    
    // 5. 返航: 回到起飞点上方
    mission_waypoints.push_back({0.0, 0.0, 2.8, POS_MODE}); 
    
    // 6. 降落准备
    mission_waypoints.push_back({0.0, 0.0, 0.0, POS_MODE}); 

    // --- 变量初始化 ---
    int wp_index = 0;
    bool mission_finished = false;
    
    // PID 参数 (用于 VEL_MODE 导航)
    // 参数含义: P, I, D, Limit(最大速度m/s)
    PIDController pid_nav_x(1.0, 0.01, 0.0, 1.0); 
    PIDController pid_nav_y(1.0, 0.01, 0.0, 1.0);
    PIDController pid_z(1.2, 0.1, 0.0, 1.0);     
    
    bool pid_reset_needed = true;
    ros::Time last_time = ros::Time::now();

    // === 1. 强制等待定位并记录初始机头 ===
    ROS_INFO("Waiting for valid local position...");
    while(ros::ok()) {
        ros::spinOnce();
        if(current_pose.header.seq > 0 && current_pose.pose.position.z > -50) break;
        rate.sleep();
    }
    // 稳定一下数据
    for(int i=0; i<30; i++) { ros::spinOnce(); rate.sleep(); }

    // 记录初始状态
    tf2::Quaternion q_start;
    tf2::fromMsg(current_pose.pose.orientation, q_start);
    double r_dummy, p_dummy;
    tf2::Matrix3x3(q_start).getRPY(r_dummy, p_dummy, initial_yaw);
    
    initial_pos_x = current_pose.pose.position.x;
    initial_pos_y = current_pose.pose.position.y;
    initial_recorded = true;

    ROS_INFO("=== HOME RECORDED ===");
    ROS_INFO("Initial Yaw: %.2f deg", initial_yaw * 180.0 / M_PI);
    ROS_INFO("Initial Pos: (%.2f, %.2f)", initial_pos_x, initial_pos_y);
    ROS_INFO("Mission: 6 Waypoints Loaded.");
    ROS_INFO("=====================");

    // 预发送 Setpoint (防止切 Offboard 瞬间掉落)
    geometry_msgs::PoseStamped pose_cmd;
    pose_cmd.pose.position.x = initial_pos_x;
    pose_cmd.pose.position.y = initial_pos_y;
    pose_cmd.pose.position.z = current_pose.pose.position.z;
    for(int i = 20; ros::ok() && i > 0; --i){
        pos_pub.publish(pose_cmd);
        ros::spinOnce(); rate.sleep();
    }

    // 设置模式相关变量
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_req = ros::Time::now();

    while(ros::ok()){
        // 计算 dt
        double dt = (ros::Time::now() - last_time).toSec();
        last_time = ros::Time::now();

        // 紧急退出
        if(current_state.mode == "POSCTL") {
            ROS_INFO("Detected POSCTL, Exiting Node.");
            break;
        }

        // 自动解锁与切 Offboard
        if(current_state.connected) {
            if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_req > ros::Duration(5.0))) {
                set_mode_client.call(offb_set_mode);
                last_req = ros::Time::now();
            } else {
                if(!current_state.armed && (ros::Time::now() - last_req > ros::Duration(5.0))) {
                    arming_client.call(arm_cmd);
                    last_req = ros::Time::now();
                }
            }
        }

        if (current_state.mode == "OFFBOARD" && current_state.armed) 
        {
            if (mission_finished) {
                // 任务结束，保持降落状态
                mavros_msgs::SetMode land_mode;
                land_mode.request.custom_mode = "AUTO.LAND";
                set_mode_client.call(land_mode);
                break;
            }

            Waypoint wp = mission_waypoints[wp_index];
            
            // 1. 获取当前姿态
            tf2::Quaternion q_curr;
            tf2::fromMsg(current_pose.pose.orientation, q_curr);
            double curr_r, curr_p, curr_yaw;
            tf2::Matrix3x3(q_curr).getRPY(curr_r, curr_p, curr_yaw);

            // 2. 将航点 "向前/向左" 转换为 "ENU X/Y"
            double target_enu_x, target_enu_y;
            transform_body_to_enu(wp.fwd, wp.lat, target_enu_x, target_enu_y);

            // 3. 判断是否到达航点
            double dist = get_distance(target_enu_x, target_enu_y, wp.z, current_pose.pose.position);
            
            // 判定阈值：水平距离 < 0.2m 且高度接近 (如果目标高度 > 0.5)
            if (dist < 0.2 && (wp.z < 0.5 || abs(current_pose.pose.position.z - wp.z) < 0.2)) {
                ROS_INFO("Reached Waypoint %d (Fwd: %.1f, Lat: %.1f)", wp_index, wp.fwd, wp.lat);
                wp_index++;
                pid_reset_needed = true; // 切换航点时重置 PID 积分
                if (wp_index >= mission_waypoints.size()) {
                    mission_finished = true;
                }
                continue; // 立即处理下一个点
            }

            // 4. 执行控制 (位置模式 OR 速度模式)
            if (wp.mode == POS_MODE) 
            {
                // --- 位置控制 ---
                geometry_msgs::PoseStamped pos_msg;
                pos_msg.pose.position.x = target_enu_x;
                pos_msg.pose.position.y = target_enu_y;
                pos_msg.pose.position.z = wp.z;
                
                // 锁头：始终指向初始方向 (Initial Yaw)
                tf2::Quaternion q; q.setRPY(0,0,initial_yaw);
                pos_msg.pose.orientation = tf2::toMsg(q);

                pos_pub.publish(pos_msg);
                pid_reset_needed = true; 
            }
            else // VEL_MODE
            {
                // --- 速度控制 (PID 导航) ---
                if(pid_reset_needed) {
                    pid_nav_x.reset(); pid_nav_y.reset(); pid_z.reset();
                    pid_reset_needed = false;
                }

                // 计算 ENU 坐标系下的速度
                double vx = pid_nav_x.calculate(target_enu_x, current_pose.pose.position.x, dt);
                double vy = pid_nav_y.calculate(target_enu_y, current_pose.pose.position.y, dt);
                double vz = pid_z.calculate(wp.z, current_pose.pose.position.z, dt);

                geometry_msgs::Twist vel_cmd;
                vel_cmd.linear.x = vx;
                vel_cmd.linear.y = vy;
                vel_cmd.linear.z = vz;

                // 锁头逻辑 (P控制器)
                double yaw_err = initial_yaw - curr_yaw;
                if(yaw_err > M_PI) yaw_err -= 2*M_PI; 
                else if(yaw_err < -M_PI) yaw_err += 2*M_PI;
                vel_cmd.angular.z = 1.0 * yaw_err;

                vel_pub.publish(vel_cmd);
            }
        }
        else {
            // 未解锁时发布当前位置占位
            geometry_msgs::PoseStamped hold;
            hold.pose = current_pose.pose;
            pos_pub.publish(hold);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}