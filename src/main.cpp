#include <ros/ros.h>
#include <signal.h>
#include "longitudinal.hh"

#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

ros::Publisher pubCommands, pubVel; // Commands & velocity profile publishers
ros::Publisher pubFinish;           // Finish flag publisher

void dynamicCallback(long_pid::longConfig &config, uint32_t level, Longitudinal* lng){

    ROS_WARN("LONG: Setting new dynamic parameters..");
	lng->reconfigure(config);

}

void my_handler(int sig){

    if(ros::master::check()){
        as_msgs::CarCommands msgCommands;
        msgCommands.motor = -1.0;
        for(int i=0; i<5; i++){
            pubCommands.publish(msgCommands);
            ros::Duration(0.05).sleep();
        }
        ROS_ERROR("LONG says Goodbye :)");
    }
    ros::shutdown();
}

int main(int argc, char **argv) {

    // Init Node:
    ros::init(argc, argv, "long_pid");

    // Handle Connections:
    ros::NodeHandle nh("~");

    // Signal handler for publishing 0s when dying
    signal(SIGINT, my_handler); // override default ros sigint signal

    // Longitudinal controller object
    Longitudinal lng = Longitudinal();

    // Get params
    string stateTopic, plannerTopic, troTopic, commandsTopic, velTopic, 
            finishTopic, preFinishTopic, modeParamTopic;

    nh.param<string>("Topics/State",            stateTopic,             "/AS/C/state");
    nh.param<string>("Topics/Planner",          plannerTopic,           "/AS/C/trajectory/partial");
    nh.param<string>("Topics/Tro",              troTopic,               "/AS/C/trajectory/full");
    nh.param<string>("Topics/Commands",         commandsTopic,          "/AS/C/motor");
    nh.param<string>("Topics/Velocities",       velTopic,               "/AS/C/pid/velocity");
    nh.param<string>("Topics/Finish",           finishTopic,            "/AS/C/flags/finish");
    nh.param<string>("Topics/ModeParameters",   modeParamTopic,         "/CTRL/ModeParameters");
    nh.param<string>("DebugFilePath",           lng.savePath,           "");

    nh.param<int>("nPlanning",  lng.nPlanning,  1900);
    nh.param<int>("Hz",         lng.freq,       40);
    nh.param<int>("mission",    lng.mission,    0);

    ROS_INFO("nPlanning: %i", lng.nPlanning);

    nh.param<bool>("TROflag", lng.TROflag, false);

    cout << "TROflag: " << lng.TROflag << endl;

    nh.param<double>("MinVelFinish",  lng.minVelFinish,   0.1);
    nh.param<double>("Delta_s",       lng.delta_s,        0.025);
    nh.param<double>("Mass",          lng.m,              240.0);
    nh.param<double>("Radius",        lng.radius,         0.2);

    double searchDist;
    nh.param<double>("nSearchAhead", searchDist, 5.0);
    lng.nSearch = (int) searchDist/lng.delta_s;

    if(lng.mission <= 2) nh.param<string>("Topics/SkiFinish", preFinishTopic, "/AS/C/skidpad/finish");
    else nh.param<string>("Topics/AccFinish", preFinishTopic, "/AS/C/acceleration/finish");
    
    // Publishers & Subscribers
    ros::Subscriber subState = nh.subscribe(stateTopic, 10, &Longitudinal::stateCallback, &lng);
    ros::Subscriber subPlanner = nh.subscribe(plannerTopic, 10, &Longitudinal::plannerCallback, &lng);
    ros::Subscriber subTro = nh.subscribe(troTopic, 10, &Longitudinal::plannerCallback, &lng);
    ros::Subscriber subPreFinish = nh.subscribe(preFinishTopic, 10, &Longitudinal::finishCallback, &lng);
    ros::Subscriber subModeParam = nh.subscribe(modeParamTopic, 10, &Longitudinal::modeParamCallback, &lng);
    
    pubCommands = nh.advertise<as_msgs::CarCommands>(commandsTopic, 1);
    pubVel = nh.advertise<as_msgs::CarVelocityArray>(velTopic, 10);
    pubFinish = nh.advertise<std_msgs::Bool>(finishTopic, 10);

        // DEBUG
    ros::Publisher pubTargetVel = nh.advertise<as_msgs::Float32Stamped>("/AS/C/pid/target/velocity", 10);
    ros::Publisher pubLatency = nh.advertise<as_msgs::Float32Stamped>("/AS/C/pid/latency", 10);
    ros::Publisher pubKp = nh.advertise<as_msgs::Float32Stamped>("/AS/C/pid/kp", 10);

    // Dynamic reconfigure
	dynamic_reconfigure::Server<long_pid::longConfig> server_lng;
	dynamic_reconfigure::Server<long_pid::longConfig>::CallbackType f;
	f = boost::bind(&dynamicCallback, _1, _2, &lng);
	server_lng.setCallback(f);

    ros::Duration(2.0).sleep();

    // Msg declaration
    as_msgs::CarCommands msg;
    as_msgs::CarVelocityArray velMsg;
    as_msgs::Float32Stamped floatMsg;
    std_msgs::Bool boolMsg;

    ros::Rate r(lng.freq);
    while(ros::ok()){

        chrono::_V2::system_clock::time_point start = chrono::system_clock::now();

        lng.run(); // run PI controller

        chrono::_V2::system_clock::time_point end = chrono::system_clock::now();
        chrono::duration<double> elapsed_time = end-start;
        cout << "LONG PID: RUN TIME: " << elapsed_time.count()*1000 << " ms" << endl;

        lng.msgCommands(&msg);
        lng.msgVelocity(&velMsg);
        if(lng.firstvelFlag){
            pubCommands.publish(msg); // Publish motor
            pubVel.publish(velMsg);
        }

        boolMsg.data = lng.isFinish();
        pubFinish.publish(boolMsg); // Publish finish flag
        
        // DEBUG
        floatMsg.header.stamp = ros::Time::now();

        floatMsg.data = lng.target_vel;
        pubTargetVel.publish(floatMsg); // Publish target velocity

        floatMsg.data = lng.latency;
        pubLatency.publish(floatMsg);

        floatMsg.data = lng.pid->Kp;
        pubKp.publish(floatMsg);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}