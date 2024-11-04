#include "ros/ros.h"
#include <ros/package.h>
#include <mutex>
#include <iostream> 
#include <gripper/Gripper_monitor.h>
#include <gripper/Gripper_target.h>
#include <geometry_msgs/WrenchStamped.h>
#include "std_srvs/Trigger.h"  // Include the Trigger service message
#include <gripper/GainTune.h>
#include <vector>

struct ft_data
{
    double fx = 0;
    double fy = 0;
    double fz = 0;
    double tx = 0;
    double ty = 0;
    double tz = 0;
};

class Gripper_closed_loop{
    private:
        
        std::mutex mtx_sensor_1;
        std::mutex mtx_sensor_2;
        bool sensor_1_active =  false;
        bool sensor_2_active = false;
        bool ft_calibrated = false;
        ft_data sensor_1;
        ft_data sensor_2;
        ft_data calibrated_sensor_1;
        ft_data calibrated_sensor_2;
        ft_data sensor_offset_1;
        ft_data sensor_offset_2;
        double target_force = 0;
        double control_force = 0;
        double Kp = 2.0; // 2
        double Ki = 100.0; // 50 
        double Kd = 0; // 0.015
        double Kd_vel = 0.0;
        double gamma = 5.0;
        double e_pre = 0;
        double theta = 0;
        double e_int = 0;
        double ki_int_max = 10;
        double pre_target_force = 0;
        double direction_pre = 0;
        ros::Duration delta_time;
        ros::Time prev_time = ros::Time::now();
        ros::Subscriber sub_gripper_state;
        ros::Subscriber sub_sensor_1;
        ros::Subscriber sub_sensor_2;
        ros::Subscriber sub_target_force;

        ros::Publisher pub_gripper_force;
        ros::Publisher pub_calibrated_sensor_1;
        ros::Publisher pub_calibrated_sensor_2;

        ros::ServiceServer calibrate_srv;
        ros::ServiceServer calibrate_gamma;
        ros::ServiceServer calibrate_ki;
        ros::ServiceServer calibrate_kp;
        ros::ServiceServer calibrate_kd_vel;


    public:
        // constructor
        Gripper_closed_loop(ros::NodeHandle *nh);
        
        void callback_gripper(const gripper::Gripper_monitor::ConstPtr& msg_data);
        void callback_target(const gripper::Gripper_target::ConstPtr& msg_data);
        
        void remove_offset(ft_data* calibrated, ft_data* non_calibrated, ft_data* offset);
        void struct_to_msg(geometry_msgs::WrenchStamped* msg, ft_data* calibrated);
        bool callibrate_sensors(std_srvs::Trigger::Request &req,
                                std_srvs::Trigger::Response &res);

        bool set_kp(gripper::GainTune::Request &req, gripper::GainTune::Response &res);
        bool set_ki(gripper::GainTune::Request &req, gripper::GainTune::Response &res);
        bool set_gamma(gripper::GainTune::Request &req, gripper::GainTune::Response &res);
        bool set_kd_vel(gripper::GainTune::Request &req, gripper::GainTune::Response &res);

        void callback_sensor_1(const geometry_msgs::WrenchStamped::ConstPtr& msg_data);

        void callback_sensor_2(const geometry_msgs::WrenchStamped::ConstPtr& msg_data);

        void ft_msg_to_struct(const geometry_msgs::WrenchStamped::ConstPtr& msg_data, ft_data* ft_data_obj);
};


Gripper_closed_loop::Gripper_closed_loop(ros::NodeHandle *nh){

    sub_gripper_state = nh->subscribe("/gripper_monitor", 2, &Gripper_closed_loop::callback_gripper, this);
    sub_sensor_1 = nh->subscribe("/netft_data1", 2, &Gripper_closed_loop::callback_sensor_1, this);
    sub_sensor_2 = nh->subscribe("/netft_data2", 2, &Gripper_closed_loop::callback_sensor_2, this);
    sub_target_force = nh->subscribe("/gripper_closed_loop", 2, &Gripper_closed_loop::callback_target, this);

    pub_gripper_force = nh->advertise<gripper::Gripper_target>("/gripper_control", 1);
    pub_calibrated_sensor_1 = nh->advertise<geometry_msgs::WrenchStamped>("/netft_data_1_calibrated", 1);
    pub_calibrated_sensor_2 = nh->advertise<geometry_msgs::WrenchStamped>("/netft_data_2_calibrated", 1);

    calibrate_srv = nh->advertiseService("/gripper_callibrate_ft", &Gripper_closed_loop::callibrate_sensors, this);
    calibrate_kp = nh->advertiseService("/gripper_set_kp", &Gripper_closed_loop::set_kp, this);
    calibrate_ki = nh->advertiseService("/gripper_set_ki", &Gripper_closed_loop::set_ki, this);
    calibrate_gamma = nh->advertiseService("/gripper_set_gamma", &Gripper_closed_loop::set_gamma, this);
    calibrate_kd_vel = nh->advertiseService("/gripper_set_kd_vel", &Gripper_closed_loop::set_kd_vel, this);

};

void Gripper_closed_loop::callback_target(const gripper::Gripper_target::ConstPtr& msg_data){
    target_force = msg_data->target;
};


void Gripper_closed_loop::callback_gripper(const gripper::Gripper_monitor::ConstPtr& msg_data){

    ros::Time time_now = ros::Time::now();
    delta_time = time_now - prev_time;
    prev_time = time_now;
    double dt = delta_time.toSec();

    double velocity = msg_data->velocity;
    double target_diff = target_force - pre_target_force;
    pre_target_force = target_force;
    double avg_force = -(calibrated_sensor_1.fz + calibrated_sensor_2.fz)/2;
    //double avg_force = -calibrated_sensor_1.fz;
    double e = 0;

    if (abs(avg_force) < 0.5 || ft_calibrated == false || target_force < 0){
        // make sure it opens and closes
        if (abs(target_force)<6){
            control_force = copysign(6, target_force);
        }{
            control_force = target_force;
        }
        e_int = 0;
        e_pre = 0;
    }
    else{
        e = target_force - avg_force;
        double dedt = pow((e - e_pre), 2);
        e_pre = e;

        e_int += e * dt;

        double ki_int = e_int * Ki; 

        //double ki_int_max_b = ki_int_max / (gamma*abs(velocity) + 1.0);
        double gamma_de = (gamma*abs(dedt) + 1.0);
        double ki_int_max_b = ki_int_max / gamma_de;

        if (abs(ki_int) > ki_int_max_b){
            e_int  = copysign(ki_int_max_b, ki_int)/(Ki + 1e-6);
            ki_int = copysign(ki_int_max_b, ki_int);
        } 



        control_force = target_force + ki_int + Kd_vel*(e - e_pre) + (Kp/gamma_de)*e; 

        //std::cout << "avg_forece " << avg_force << " error " << e << "kd_int" << kd_int << std::endl;
    }

    gripper::Gripper_target msg;
    msg.header.stamp = ros::Time::now();
    msg.target = control_force;
    pub_gripper_force.publish(msg);
};

/*
void Gripper_closed_loop::callback_gripper(const gripper::Gripper_monitor::ConstPtr& msg_data){
    double msg_target_force = msg_data->target_force;
    double velocity = msg_data->velocity;
    double avg_force = -calibrated_sensor_1.fz;
    double delta_f = 0;

    ros::Time time_now = ros::Time::now();
    delta_time = time_now - prev_time;
    prev_time = time_now;
    double dt = delta_time.toSec();

    if (abs(avg_force) < 1 || ft_calibrated == false || target_force < 0){
        control_force = target_force;
        e_int = 0;
        e_pre = 0;
        theta = 0;
    }
    else{
        delta_f = target_force - avg_force;
        e_int += delta_f * dt;

        double ki_int = e_int * Ki; 
        double theta_hat_dot = -gamma*velocity*delta_f;
        theta += theta_hat_dot*dt;    

        control_force = target_force + ki_int + Kd_vel*velocity + theta*velocity; 
    }

    gripper::Gripper_target msg;
    msg.header.stamp = ros::Time::now();
    msg.target = control_force;
    pub_gripper_force.publish(msg);
};
*/


void Gripper_closed_loop::callback_sensor_1(const geometry_msgs::WrenchStamped::ConstPtr& msg_data){
    sensor_1_active = true;
    mtx_sensor_1.lock();
    this->ft_msg_to_struct(msg_data, &sensor_1);
    mtx_sensor_1.unlock();
    this->remove_offset(&calibrated_sensor_1, &sensor_1, &sensor_offset_1);
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = msg_data->header.stamp;
    this->struct_to_msg(&msg, &calibrated_sensor_1);
    pub_calibrated_sensor_1.publish(msg);
};

void Gripper_closed_loop::struct_to_msg(geometry_msgs::WrenchStamped* msg, ft_data* calibrated){
    msg->wrench.force.x = calibrated->fx;
    msg->wrench.force.y = calibrated->fy;
    msg->wrench.force.z = calibrated->fz;

    msg->wrench.torque.x = calibrated->tx;
    msg->wrench.torque.y = calibrated->ty;
    msg->wrench.torque.z = calibrated->tz;
}

void Gripper_closed_loop::callback_sensor_2(const geometry_msgs::WrenchStamped::ConstPtr& msg_data){
    sensor_2_active = true;
    mtx_sensor_2.lock();
    this->ft_msg_to_struct(msg_data, &sensor_2);
    mtx_sensor_2.unlock();
    this->remove_offset(&calibrated_sensor_2, &sensor_2, &sensor_offset_2);
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = msg_data->header.stamp;
    this->struct_to_msg(&msg, &calibrated_sensor_2);
    pub_calibrated_sensor_2.publish(msg);
};

void Gripper_closed_loop::remove_offset(ft_data* calibrated, ft_data* non_calibrated, ft_data* offset){
    calibrated->fx = non_calibrated->fx - offset->fx;
    calibrated->fy = non_calibrated->fy - offset->fy;
    calibrated->fz = non_calibrated->fz - offset->fz;

    calibrated->tx = non_calibrated->tx - offset->tx;
    calibrated->ty = non_calibrated->ty - offset->ty;
    calibrated->tz = non_calibrated->tz - offset->tz;
}

void Gripper_closed_loop::ft_msg_to_struct(const geometry_msgs::WrenchStamped::ConstPtr& msg_data, ft_data* ft_data_obj){
    ft_data_obj->fx = msg_data->wrench.force.x;
    ft_data_obj->fy = msg_data->wrench.force.y;
    ft_data_obj->fz = msg_data->wrench.force.z;

    ft_data_obj->tx = msg_data->wrench.torque.x;
    ft_data_obj->ty = msg_data->wrench.torque.y;
    ft_data_obj->tz = msg_data->wrench.torque.z;
}

bool Gripper_closed_loop::callibrate_sensors(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

    if (sensor_1_active != true || sensor_2_active != true){
        res.success = false;
        res.message = "Both sensors are not active";
        return true;
    }
    ft_calibrated == false;
    

    int num_samples = 50;
    // reset values
    sensor_offset_1 = {};
    sensor_offset_2 = {};
    ros::Rate loop_rate(500);
    ft_calibrated == false;
    loop_rate.sleep();
    loop_rate.sleep();
    for (int i = 0; i < num_samples; i++){
        mtx_sensor_1.lock();
        sensor_offset_1.fx += sensor_1.fx/num_samples;
        sensor_offset_1.fy += sensor_1.fy/num_samples;
        sensor_offset_1.fz += sensor_1.fz/num_samples;

        sensor_offset_1.tx += sensor_1.tx/num_samples;
        sensor_offset_1.ty += sensor_1.ty/num_samples;
        sensor_offset_1.tz += sensor_1.tz/num_samples;
        mtx_sensor_1.unlock();
        mtx_sensor_2.lock();
        sensor_offset_2.fx += sensor_2.fx/num_samples;
        sensor_offset_2.fy += sensor_2.fy/num_samples;
        sensor_offset_2.fz += sensor_2.fz/num_samples;

        sensor_offset_2.tx += sensor_2.tx/num_samples;
        sensor_offset_2.ty += sensor_2.ty/num_samples;
        sensor_offset_2.tz += sensor_2.tz/num_samples;
        mtx_sensor_2.unlock();

        loop_rate.sleep();
    }

    res.success = true;
    res.message = "Calibration done, bias has been compensated";
    ft_calibrated = true;
    return true;
}

bool Gripper_closed_loop::set_kp(gripper::GainTune::Request &req, gripper::GainTune::Response &res){
    Kp = req.value;
    std::cout << "Kp value " << Kp << std::endl;

    res.success = true; 
    return true;
}

bool Gripper_closed_loop::set_ki(gripper::GainTune::Request &req, gripper::GainTune::Response &res){
    Ki = req.value;
    std::cout << "Ki value " << Ki << std::endl;

    res.success = true; 
    return true;
}

bool Gripper_closed_loop::set_gamma(gripper::GainTune::Request &req, gripper::GainTune::Response &res){
    gamma = req.value;
    std::cout << "Gamma value " << gamma << std::endl;

    res.success = true; 
    return true;
}

bool Gripper_closed_loop::set_kd_vel(gripper::GainTune::Request &req, gripper::GainTune::Response &res){
    Kd_vel = req.value;
    std::cout << "Kd vel value " << Kd_vel << std::endl;

    res.success = true; 
    return true;
}


int main(int argc, char** argv){
    // ROS
    ros::init(argc, argv, "gripper_closed_loop_fc");
    ros::NodeHandle nh;
    // multithreaded spinner 
    ros::AsyncSpinner spinner(0);
    spinner.start();
    // main class 
    Gripper_closed_loop gipper_control(&nh); 

    ros::waitForShutdown();

    return 0;
}











