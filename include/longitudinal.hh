/*
 Copyright (c) 2024 Oriol Martínez @fetty31

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef LONG_HH
#define LONG_HH

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include <string>
#include <vector>
#include <stdio.h>
#include <time.h>
#include <eigen3/Eigen/Dense>
#include <regex>
#include "lapcount.hh"
#include "pid.hh"

// Dynamic reconfigure headers
#include <dynamic_reconfigure/server.h>
#include <long_pid/longConfig.h>

// Msgs used
#include "as_msgs/ObjectiveArrayCurv.h"
#include "as_msgs/CarState.h"
#include "as_msgs/CarCommands.h"
#include "as_msgs/CarVelocityArray.h"
#include "as_msgs/Float32Stamped.h"
#include "ctrl_msgs/ModeParameters.h"

#include "std_msgs/Bool.h"

using namespace std;

class Longitudinal{

    private:

        int firstLatency = 5;                               // First latency after recomputing velocity profile

        double ax_acc_max = 8., ax_dec_max = 10.;           // Maximum longitudinal acceleration
        double ay_max = 10.;                                // Maximum lateral acceleration
        double ax_mode = 8., ax_dec_mode = 10.;             // Longitudinal acceleration limit from LLC
        double spacing = 0.025;                             // Planner discretization
        double vx_max = 20., vx_final = 12., vx_min = 2.;   // velocity restrictions
        double lluisrho = 1.;                               // > 1 for increasing velocity profile, < 1 for decreasing (0 < lluisrho < inf)
        double throttle = 0.;
        double Kp = 1.0;                                    // PID constants
        const double Ki = 1.0;
        const double Kd = 1.0;             
        double current_s = 0.;                              // Current progress of the car 
        double troProfileMU = 1.;                           // Offline velocity profile factor (0 < mu <= 1)  
        double minDist = 15.;                               // Minimum distance for recomputing velocity profile
        double smax = 0.;                                   // Maximum progress seen

        bool posFlag = false, velFlag = false;              // New position && new velocity flags
        bool lastApex = false;                              // Whether to pick accelerating profile from last apex to the end (true) or picking a final velocity (false)                
        bool stateFlag = false, dynParamFlag = false, modeParamFlag = false;
        bool spatialFlag = true;                            // Whether we use the space domain velocity profile or not
        bool finished = false;                              // Finish flag
        bool preFinish = false;                             // Planner finish flag (pre-finish flag)
        bool TrqLimited = false;                            // Whether the controller will run torque limited

        chrono::_V2::system_clock::time_point tic;
        chrono::_V2::system_clock::time_point tac;

        void online_planning(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg);  // Planner Callback aux functions
        void offline_planning(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg);

        double vel2acc(double &vf, double &vi, int &lat, int &pre_lat, double &delta_x);
        double trq2acc(double &trq, double &mass, double &radius);
        
        void spatialVelProf();                          // Computes a velocity profile solving the gg EDO dv/ds = ax_max/v * sqrt(1 - (v²/ay_max*R)²) --> space dependent
        void timeVelProf();                             // Computes a velocity profile with classic MRUA + gg --> time dependent

        double f_accel(int k, double v);                // Aux functions for spatial velocity profile
        double f_decel(int k, double v);

        bool time2replan();                             // Returns true when the car is near the end of the recieved trajectory
        bool time2reset();                              // Whether we must reset the integral term from PID (necessary in stand still)

        vector<int> findLocalMax(Eigen::VectorXd curv); // Returns a vector with curvature apexes index
        template<typename mytype> void printVec(vector<mytype> &input, int firstElements);
        int first_index(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg);
        void update_kp(double &ax, double &limit_ax);
        const string currentDateTime(); // get current date/time, format is YYYY-MM-DD.HH:mm:ss

        // Lapcount object
        Lapcount* lapcount;

    public:

        Longitudinal();
        ~Longitudinal();
        void stateCallback(const as_msgs::CarState::ConstPtr& msg);
        void plannerCallback(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg);
        void finishCallback(const std_msgs::Bool::ConstPtr& msg);
        void modeParamCallback(const ctrl_msgs::ModeParameters::ConstPtr& msg);
        void reconfigure(long_pid::longConfig& config);
        void msgCommands(as_msgs::CarCommands *msg);
        void msgVelocity(as_msgs::CarVelocityArray *msg);
        void saveEigen(string filePath, string name, Eigen::MatrixXd data, bool erase);
        template<typename mytype> void save(string filePath, string name, mytype data, bool time, bool unique=false);
        bool isFinish();

        void run(); // run PID controller

        // PID object
        PID* pid;

        int freq = 40;          // Frequency of PID controller
        int mission = 0;        // 0 for AX, 1 for TD, 2 for SkidPad, 3 for Acceleration
        int nPlanning = 10;     // Minimum planner points
        int latency = 0;        // Latency of the system
        int nSearch = 200;      // Number of points where we will look for the closest point to the car

        string savePath;  // Path to save debug data

        bool firstvelFlag = false;  // First velocity profile flag
        bool TROflag = false;       // If true, we will follow TRO's velocity profile

        // Planner's trajectory matrix 
        Eigen::MatrixXd planner; // [x, y, s, k]

        //Actual state of the car
        Eigen::VectorXd carState; // [x, y, theta, vx, vy, w, delta(steering), ax, ay, Mtv]

        // Velocity profile vector
        Eigen::VectorXd velocities;
        Eigen::VectorXd accelerations;

        double target_vel = 0;
        double minVelFinish = 0.05;   // Minimum velocity to publish finish flag when laps are completed
        double delta_s = 0.025;       // planner discretization
        double m = 240, radius = 0.2; // Car's mass & wheel radius
};


#endif