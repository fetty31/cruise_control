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

#ifndef PID_HH
#define PID_HH

#include <iostream>
#include <cmath>
#include <stdio.h>
#include <chrono>
#include <time.h>

class PID{

    public:
        PID(double dt, double max, double min, double Kp, double Ki, double Kd);    // Constructor with constant dt
        PID(double max, double min, double Kp, double Ki, double Kd);               // Constructor with variable dt
        ~PID();                                                                     // Destructor

        double calculate(double setpoint, double cv);               // Calculates PID output given a setpoint and current value
        double calculate_anti_windup(double setpoint, double cv);   // Calculates PID output with anti-windup
        void reset_integral();                                      // Resets integral term

        double Kp, Ki, Kd;   // PID constants

    private:
        double _max, _min;      // max, min values for control var
        double _pre_error;      // Previous error
        double _integral;       // Integral var
        double _dt;       // Time interval
        std::chrono::_V2::system_clock::time_point tic;
        std::chrono::_V2::system_clock::time_point tac;
        bool _first_iter;

        double run(double &setpoint, double &cv, double &dt); // runs PID with given dt
        double run(double &setpoint, double &cv);             // runs PID with variable dt

        double run_anti_windup(double &setpoint, double &cv, double &dt); // runs PID with given dt
        double run_anti_windup(double &setpoint, double &cv);             // runs PID with variable dt
};


#endif