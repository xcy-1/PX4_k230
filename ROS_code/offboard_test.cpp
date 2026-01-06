/**
 * @file mixed_control_node.cpp
 * @brief 混合控制 + 视觉状态机 (基于起飞航向的相对坐标系版)
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>
#include <serial/serial.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

// --- 全局配置 ---
const int img_center_x_ = 320; 
const int img_center_y_ = 280;//微调位置，将投放物体位于正中心

// === 修改点 1: 全局变量存储初始状态 ===
double initial_yaw = 0.0;       // 起飞时的机头朝向
double initial_pos_x = 0.0;     // 起飞时的ENU X坐标
double initial_pos_y = 0.0;     // 起飞时的ENU Y坐标
bool initial_recorded = false;  // 标记是否已记录

// --- 视觉数据结构 ---
#pragma pack(1)
struct VisionFrame {
    uint8_t header[2];
    uint8_t has_object;
    uint8_t obj_class;
    int16_t cx; int16_t cy;
    uint32_t area;
    uint8_t footer;
};
#pragma pack()

// --- 枚举定义 ---
enum ControlMode { POS_MODE, VEL_MODE };

// 视觉任务状态机
enum VisualTaskState {
    VIS_IDLE,       
    VIS_ALIGN_HIGH, 
    VIS_DESCEND,    
    VIS_ALIGN_LOW,  
    VIS_COMPLETE    
};

struct Waypoint {
    double fwd; // === 修改: x 改名为 fwd (向前距离)
    double lat; // === 修改: y 改名为 lat (向左距离)
    double z;
    ControlMode mode;
    bool enable_vision; 
};

// --- 全局变量 ---
bool has_target_ = false;
int16_t target_cx_ = 0, target_cy_ = 0;
serial::Serial ser_;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

// --- PID 类 (保持不变) ---
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
        if (error_sum > limit/ki) error_sum = limit/ki;
        if (error_sum < -limit/ki) error_sum = -limit/ki;
        double derivative = (error - error_last) / dt;
        error_last = error;
        double output = kp * error + ki * error_sum + kd * derivative;
        return std::max(std::min(output, limit), -limit);
    }
    void reset() { error_last = 0; error_sum = 0; }
};

// --- 回调函数 ---
void state_cb(const mavros_msgs::State::ConstPtr& msg){ current_state = *msg; }
void pose_cb (const geometry_msgs::PoseStamped::ConstPtr& msg){ current_pose = *msg; }

double get_distance(double x, double y, double z, const geometry_msgs::Point& p) {
    return sqrt(pow(x - p.x, 2) + pow(y - p.y, 2) + pow(z - p.z, 2));
}

// === 修改点 2: 新增坐标转换函数 (Body -> ENU) ===
// 输入: 向前距离(fwd), 向左距离(lat)
// 输出: ENU坐标系下的绝对 X, Y (引用传递)
void transform_body_to_enu(double fwd, double lat, double &out_x, double &out_y) {
    if (!initial_recorded) {
        // 如果还没记录初始点，就默认原点
        out_x = fwd; 
        out_y = lat;
        return;
    }
    // 旋转矩阵公式 (顺时针旋转 coordinate system 等价于逆时针旋转 vector)
    // x_enu = x_body * cos(theta) - y_body * sin(theta)
    // y_enu = x_body * sin(theta) + y_body * cos(theta)
    // 加上初始偏移量 initial_pos
    out_x = initial_pos_x + fwd * cos(initial_yaw) - lat * sin(initial_yaw);
    out_y = initial_pos_y + fwd * sin(initial_yaw) + lat * cos(initial_yaw);
}

// 简单的串口读取 (保持不变)
void read_serial_data() {
    if (ser_.available()) {
        std::vector<uint8_t> buffer;
        ser_.read(buffer, ser_.available());
        if (buffer.size() < 13) return; 
        for (size_t i = 0; i <= buffer.size() - 13; ++i) { 
            if (buffer[i] == 0xAA && buffer[i+1] == 0x55 && buffer[i+12] == 0xED) {
                has_target_ = (buffer[i+2] == 1);
                if (has_target_) {
                    target_cx_ = (int16_t)((buffer[i+4] << 8) | buffer[i+5]);
                    target_cy_ = (int16_t)((buffer[i+6] << 8) | buffer[i+7]);
                    ROS_INFO("[VISION] Target FOUND! Cx: %d, Cy: %d", target_cx_, target_cy_);
                } else {
                    ROS_INFO_THROTTLE(1.0, "[VISION] Searching... (No Object)");
                }
                break; 
            }
        }
    }
}

// --- 视觉速度计算函数 (保持不变) ---
void compute_visual_velocity(PIDController &pid_fwd, PIDController &pid_lat, 
                                double dt, double current_yaw, 
                                double &out_vx, double &out_vy) 
{
    // 这里使用 current_yaw 是正确的，因为摄像头是跟着飞机当前姿态转的
    double error_img_fwd = (double)(img_center_y_ - target_cy_); 
    double error_img_lat = (double)(img_center_x_ - target_cx_);

    double v_body_fwd = pid_fwd.calculate(0.0, -error_img_fwd, dt); 
    double v_body_lat = pid_lat.calculate(0.0, -error_img_lat, dt);

    out_vx = v_body_fwd * cos(current_yaw) - v_body_lat * sin(current_yaw);
    out_vy = v_body_fwd * sin(current_yaw) + v_body_lat * cos(current_yaw);
}

// ================= MAIN =================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mixed_control_node");
    ros::NodeHandle nh;

    // 串口初始化
    try {
        ser_.setPort("/dev/ttyUSB1"); // === 注意: 请确认是 USB0 还是 USB1
        ser_.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(50);
        ser_.setTimeout(to);
        ser_.open();
    } catch (serial::IOException& e) {
        ROS_ERROR("无法打开串口");
    }

    // ROS 通信
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);
    while(ros::ok() && !current_state.connected){ ros::spinOnce(); rate.sleep(); }

    // === 修改点 3: 航点定义 (现在是相对于起飞朝向的坐标) ===
    // {向前(m), 向左(m), 高度(m), 模式, 视觉开关}
    std::vector<Waypoint> mission_waypoints;
    
    // 0. 起飞 (原地升空)
    mission_waypoints.push_back({0.0, 0.0, 2.8, POS_MODE, false}); 
    
    // 1. 飞向点1: 向前 6.3m, 向左 2.0m (对应你原来的 6.3, 2.0)
    mission_waypoints.push_back({6.3, 2.0, 2.8, POS_MODE, false});
    
    // 2. 飞向点2: 向前 8.9m, 向左 2.0m (视觉搜索段)
    mission_waypoints.push_back({8.9, 2.0, 2.8, VEL_MODE, true});
    
    mission_waypoints.push_back({8.9, 2.0, 2.8, POS_MODE, false});// 冗余点，确保位置模式稳定



    // 3. 飞向点3: 向前 8.9m, 向右 2.0m (即向左 -2.0)
    mission_waypoints.push_back({8.9, -2.0, 2.8, POS_MODE, false});
    
    // 4. 飞向点4: 向前 6.3m, 向右 2.0m
    mission_waypoints.push_back({6.3, -2.0, 2.8, VEL_MODE, true});

    mission_waypoints.push_back({6.3, -2.0, 2.8, POS_MODE, false});// 冗余点，确保位置模式稳定
    
    // 5. 返航: 回到起飞点上方
    mission_waypoints.push_back({0.0, 0.0, 2.8, POS_MODE, false}); 
    mission_waypoints.push_back({0.0, 0.0, 1.5, POS_MODE, false}); 
    mission_waypoints.push_back({0.0, 0.0, 0.0, POS_MODE, false}); 

    // --- 变量初始化 ---
    int wp_index = 0;
    VisualTaskState vis_state = VIS_IDLE; 
    ros::Time align_start_time(0); 
    ros::Time last_sight_time(0);
    
    PIDController pid_nav_x(1.0, 0.0, 0.0, 1.0); 
    PIDController pid_nav_y(1.0, 0.0, 0.0, 1.0);
    PIDController pid_z(1.2, 0.1, 0.0, 1.0);     
    
    PIDController pid_vis_fwd(0.003, 0.0, 0.001, 0.5); 
    PIDController pid_vis_lat(0.003, 0.0, 0.001, 0.5);

    bool pid_reset_needed = true;
    bool mission_finished = false;

    // === 修改点 4: 强制等待并记录初始航向 ===
    ROS_INFO("Waiting for valid local position...");
    while(ros::ok()) {
        ros::spinOnce();
        // 确保收到过位置消息且非空
        if(current_pose.header.seq > 0) break;
        rate.sleep();
    }
    
    // 稍微延时一点，确保数据稳定
    for(int i=0; i<20; i++) { ros::spinOnce(); rate.sleep(); }

    tf2::Quaternion q_start;
    tf2::fromMsg(current_pose.pose.orientation, q_start);
    double r_dummy, p_dummy;
    tf2::Matrix3x3(q_start).getRPY(r_dummy, p_dummy, initial_yaw);
    
    initial_pos_x = current_pose.pose.position.x;
    initial_pos_y = current_pose.pose.position.y;
    initial_recorded = true;

    ROS_INFO("=== RECORDED HOME ===");
    ROS_INFO("Initial Yaw: %.2f deg", initial_yaw * 180.0 / M_PI);
    ROS_INFO("Initial Pos: x=%.2f, y=%.2f", initial_pos_x, initial_pos_y);
    ROS_INFO("The drone assumes FORWARD is %.2f deg compass.", initial_yaw * 180.0 / M_PI);
    ROS_INFO("=====================");

   
    // 预热：发送初始位置设定点 (防止切 Offboard 失败)
    geometry_msgs::PoseStamped pose_cmd;
    pose_cmd.pose.position.x = initial_pos_x;
    pose_cmd.pose.position.y = initial_pos_y;
    pose_cmd.pose.position.z = current_pose.pose.position.z;
    
    for(int i = 50; ros::ok() && i > 0; --i){
        pos_pub.publish(pose_cmd);
        ros::spinOnce();
        rate.sleep();
    } 
   
    // 设置 Offboard 模式等...
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_req_mode = ros::Time::now();
    ros::Time last_req_arm = ros::Time::now();
    ros::Time last_time = ros::Time::now();

    ROS_INFO("Mission Start Loop!");

    while(ros::ok()){

        // 紧急退出条件: 检测到切换到 POSCTL 模式
        if( current_state.mode == "POSCTL") {
            ROS_INFO("Detected POSCTL mode, exiting node.");
            break;
        }

        read_serial_data(); 

        // 自动解锁与切模式逻辑 (保持原样)
        if(current_state.connected) {
            if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_req_mode > ros::Duration(5.0))) {
                if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard enabled");
                }
                last_req_mode = ros::Time::now();
            } 
            if(!current_state.armed && (ros::Time::now() - last_req_arm > ros::Duration(5.0))) {
                if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_req_arm = ros::Time::now();
            }
        }

        double dt = (ros::Time::now() - last_time).toSec();
        last_time = ros::Time::now();

        if (current_state.mode == "OFFBOARD" && current_state.armed) 
        {
            Waypoint raw_wp = mission_waypoints[wp_index];
            
            // === 修改点 5: 实时计算当前航点的绝对 ENU 坐标 ===
            // 无论 POS_MODE 还是 VEL_MODE，我们都需要知道目标在 ENU 下的具体坐标
            double target_enu_x, target_enu_y;
            transform_body_to_enu(raw_wp.fwd, raw_wp.lat, target_enu_x, target_enu_y);

            // 获取当前机体偏航
            tf2::Quaternion q_curr;
            tf2::fromMsg(current_pose.pose.orientation, q_curr);
            double curr_roll, curr_pitch, curr_yaw;
            tf2::Matrix3x3(q_curr).getRPY(curr_roll, curr_pitch, curr_yaw);

            // =========================================================
            // === 视觉介入逻辑 (基本保持不变) ===
            // =========================================================
            if (raw_wp.enable_vision && has_target_ && vis_state == VIS_IDLE) {
                vis_state = VIS_ALIGN_HIGH;
                ROS_INFO("Target Detected! Switching to Visual Align (High).");
                last_sight_time = ros::Time::now();
            }

            if (vis_state != VIS_IDLE) 
            {
                if (has_target_) {
                    last_sight_time = ros::Time::now();
                } else {
                    if ((ros::Time::now() - last_sight_time) > ros::Duration(10.0)) {
                        ROS_WARN("Visual Target LOST! Resuming navigation...");
                        vis_state = VIS_IDLE;
                        pid_reset_needed = true;
                        continue;
                    }
                }

                double vx_vis = 0, vy_vis = 0, vz_cmd = 0;
                double target_z = 0;
                compute_visual_velocity(pid_vis_fwd, pid_vis_lat, dt, curr_yaw, vx_vis, vy_vis);

                switch (vis_state) {
                    case VIS_ALIGN_HIGH:
                        target_z = raw_wp.z; 
                        if (abs(img_center_x_ - target_cx_) < 30 && abs(img_center_y_ - target_cy_) < 30) {
                            ROS_INFO("High Align Done. Descending...");
                            vis_state = VIS_DESCEND;
                        }
                        break;
                    case VIS_DESCEND:
                        target_z = 1.5; 
                        if (abs(current_pose.pose.position.z - 1.5) < 0.1) {
                            ROS_INFO("Reached 1.5m. Fine Aligning...");
                            vis_state = VIS_ALIGN_LOW;
                            align_start_time = ros::Time::now(); // 重置计时器
                        }
                        break;
                    case VIS_ALIGN_LOW:
                        target_z = 1.5;
                        if ((ros::Time::now() - align_start_time) > ros::Duration(10.0)) {
                            ROS_INFO("Task Complete! Moving to NEXT Waypoint.");
                            vis_state = VIS_IDLE;
                            wp_index++;
                            if (wp_index >= mission_waypoints.size()) {
                                mission_finished = true;
                                wp_index = mission_waypoints.size() - 1;
                            }
                            pid_reset_needed = true;
                        } 
                        break;
                }

                vz_cmd = pid_z.calculate(target_z, current_pose.pose.position.z, dt);
                
                geometry_msgs::Twist vel_cmd;
                vel_cmd.linear.x = vx_vis;
                vel_cmd.linear.y = vy_vis;
                vel_cmd.linear.z = vz_cmd;
                
                // 锁头逻辑: 始终指向初始设定好的 forward 方向，而不是乱转
                double yaw_err = initial_yaw - curr_yaw;
                if(yaw_err > M_PI) yaw_err -= 2*M_PI; else if(yaw_err < -M_PI) yaw_err += 2*M_PI;
                vel_cmd.angular.z = 1.0 * yaw_err;

                vel_pub.publish(vel_cmd);
            }
            // =========================================================
            // === 普通航点导航逻辑 (已修改为使用 target_enu_x/y) ===
            // =========================================================
            else 
            {
                // 判断是否到达航点 (使用计算好的 ENU 目标坐标)
                double dist = get_distance(target_enu_x, target_enu_y, raw_wp.z, current_pose.pose.position);
                
                if (!mission_finished && dist < 0.2) {
                    ROS_INFO("Waypoint %d Reached (Local: %.2f, %.2f)", wp_index, raw_wp.fwd, raw_wp.lat);
                    wp_index++;
                    pid_reset_needed = true;
                    if (wp_index >= mission_waypoints.size()) {
                        mission_finished = true; 
                        wp_index = mission_waypoints.size() - 1;
                    }
                    vis_state = VIS_IDLE; 
                }

                if (raw_wp.mode == POS_MODE) {
                    geometry_msgs::PoseStamped pos_msg;
                    // === 关键: 发布的是转换后的 ENU 坐标 ===
                    pos_msg.pose.position.x = target_enu_x;
                    pos_msg.pose.position.y = target_enu_y;
                    pos_msg.pose.position.z = raw_wp.z;
                    
                    // 锁头: 保持初始朝向
                    tf2::Quaternion q; q.setRPY(0,0,initial_yaw);
                    pos_msg.pose.orientation = tf2::toMsg(q);
                    
                    pos_pub.publish(pos_msg);
                    pid_reset_needed = true;
                } 
                else { // VEL_MODE
                    if(pid_reset_needed) { 
                        pid_nav_x.reset(); pid_nav_y.reset(); pid_z.reset(); pid_reset_needed = false; 
                    }
                    // === 关键: PID 的目标也是转换后的 ENU 坐标 ===
                    // 虽然是速度控制，但我们要向 ENU 目标点飞行
                    double vx = pid_nav_x.calculate(target_enu_x, current_pose.pose.position.x, dt);
                    double vy = pid_nav_y.calculate(target_enu_y, current_pose.pose.position.y, dt);
                    double vz = pid_z.calculate(raw_wp.z, current_pose.pose.position.z, dt);
                    
                    geometry_msgs::Twist v_msg;
                    v_msg.linear.x = vx; v_msg.linear.y = vy; v_msg.linear.z = vz;
                    
                    // 锁头
                    double yaw_err = initial_yaw - curr_yaw;
                    if(yaw_err > M_PI) yaw_err -= 2*M_PI; else if(yaw_err < -M_PI) yaw_err += 2*M_PI;
                    v_msg.angular.z = 1.0 * yaw_err; 
                    
                    vel_pub.publish(v_msg);
                }
            }
        }
        else {
            // 未解锁时，发送当前位置以防 Failsafe
            geometry_msgs::PoseStamped hold;
            hold.pose = current_pose.pose;
            pos_pub.publish(hold);
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}


