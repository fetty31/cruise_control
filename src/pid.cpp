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

#include "pid.hh"

PID::PID(double dt, double max, double min, double Kp, double Ki, double Kd) :
    _dt(dt), 
    _max(max),
    _min(min),
    Kp(Kp),
    Ki(Ki),
    Kd(Kd),
    _pre_error(0.0),
    _integral(0.0),
    _first_iter(false)
{
}

PID::PID(double max, double min, double Kp, double Ki, double Kd) :
    _dt(0.0), 
    _max(max),
    _min(min),
    Kp(Kp),
    Ki(Ki),
    Kd(Kd),
    _pre_error(0.0),
    _integral(0.0),
    _first_iter(false)
{
}

PID::~PID(){}

double PID::calculate(double setpoint, double cv){ // Apply PID

    if(_dt == 0.0) return run(setpoint, cv);
    else return run(setpoint, cv, _dt);

}

double PID::calculate_anti_windup(double setpoint, double cv){ // Apply PID with anti-windup

    if(_dt == 0.0) return run_anti_windup(setpoint, cv);
    else return run_anti_windup(setpoint, cv, _dt);

}

double PID::run(double &setpoint, double &cv, double &dt){

    // Proportional
    double error = setpoint - cv;
    double Pout = Kp*error;

    // Integral
    _integral += error*dt;
    double Iout = Ki*_integral;

    // Derivative
    double deriv = (error - _pre_error)/dt;
    double Dout = Kd*deriv;

    // Final control
    double output = Pout + Iout + Dout; 

    if(output > _max) output = _max;
    else if (output < _min) output = _min;
    
    _pre_error = error;
    return output;
}

double PID::run(double &setpoint, double &cv){

    if( !_first_iter ){ // we loose first iter because of freq estimation
        this->tac = std::chrono::system_clock::now();
        _first_iter = true;
        return 0.0;
    }

    this->tic = std::chrono::system_clock::now();
    std::chrono::duration<double> dt = (tic-tac); // elapsed time between iterations
    this->tac = this->tic;

    // Proportional
    double error = setpoint - cv;
    double Pout = Kp*error;

    // Integral
    _integral += error*dt.count();
    double Iout = Ki*_integral;

    // Derivative
    double deriv = (error - _pre_error)/dt.count();
    double Dout = Kd*deriv;

    // Final control
    double output = Pout + Iout + Dout; 

    if(output > _max) output = _max;
    else if (output < _min) output = _min;
    
    _pre_error = error;
    return output;
}

double PID::run_anti_windup(double &setpoint, double &cv, double &dt){

    // Proportional
    double error = setpoint - cv;
    double Pout = Kp*error;

    // Integral
    double integral = _integral + error*dt;
    double Iout = Ki*integral;

    // Derivative
    double deriv = (error - _pre_error)/dt;
    double Dout = Kd*deriv;

    // Final control
    double output = Pout + Iout + Dout; 

    if(output > _max) output = _max;
    else if (output < _min) output = _min;
    else _integral = integral;
    
    _pre_error = error;
    return output;
}

double PID::run_anti_windup(double &setpoint, double &cv){

    if( !_first_iter ){ // we loose first iter because of freq estimation
        this->tac = std::chrono::system_clock::now();
        _first_iter = true;
        return 0.0;
    }

    this->tic = std::chrono::system_clock::now();
    std::chrono::duration<double> dt = (tic-tac); // elapsed time between iterations
    this->tac = this->tic;

    // Proportional
    double error = setpoint - cv;
    double Pout = Kp*error;

    // Integral
    double integral = _integral + error*dt.count();
    double Iout = Ki*integral;

    // Derivative
    double deriv = (error - _pre_error)/dt.count();
    double Dout = Kd*deriv;

    // Final control
    double output = Pout + Iout + Dout; 

    if(output > _max) output = _max;
    else if (output < _min) output = _min;
    else _integral = integral;
    
    _pre_error = error;
    return output;
}

void PID::reset_integral(){
    this->_integral = 0.0;
}