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

#include "longitudinal.hh"

// Constructor
Longitudinal::Longitudinal(){
    carState = Eigen::VectorXd::Zero(10);
    lapcount = new Lapcount();
    pid = new PID(1.0, -1.0, this->Kp, this->Ki, this->Kd); // variable dt
}

// Destructor
Longitudinal::~Longitudinal(){ // delete ptrs
    delete lapcount;
    delete pid; 
}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Callback functions--------------------------------------------------

void Longitudinal::stateCallback(const as_msgs::CarState::ConstPtr& msg){

    // Update car state vector with new state
    carState << msg->odom.position.x, msg->odom.position.y, msg->odom.heading, msg->odom.velocity.x, msg->odom.velocity.y, msg->odom.velocity.w, msg->steering, msg->odom.acceleration.x, msg->odom.acceleration.y, msg->Mtv;

    // Lapcount
    lapcount->setPosition(carState.head(3)); // Update lapcount's position
    if(!stateFlag || (bool) (msg->odom.velocity.x < this->minVelFinish && !this->firstvelFlag) ) lapcount->getStartingLine();  // Compute lapcount's starting line
    lapcount->run(); // counting laps
    stateFlag = true;

}

void Longitudinal::plannerCallback(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg){

    bool start = stateFlag && dynParamFlag && modeParamFlag;

    if (msg->objectives.size() < nPlanning){
        ROS_WARN("Longitudinal::Planner is too short!");
        return;
    }

    if(!start) return; // we are not ready yet

    if(this->TROflag){ // if TRO flag is active (or planner is publishing vel. profile), we want to follow the given velocity profile (with a security factor)

        offline_planning(msg); // fill planner & velocity matrices

        firstvelFlag = velFlag = true;

    }else if(msg->replan || time2replan()){ /* in autoX (or skidpad/acc), if palanner has replanned or we are near 
                                            the end of the last trajectory we got --> compute a new vel. profile */

        online_planning(msg); // fill planner matrix
        
        if(this->spatialFlag) spatialVelProf();
        else timeVelProf();
        firstvelFlag = velFlag = true;

        if(planner(planner.rows()-1, 2) > smax) smax = planner(planner.rows()-1, 2); // save max progress

    }else{ // if palanner hasn't changed & we are far away from its last point --> update current progress of the car
        current_s = msg->objectives[0].s;
        this->posFlag = true;
    }
}

void Longitudinal::finishCallback(const std_msgs::Bool::ConstPtr& msg){
    this->preFinish = msg->data;
}

void Longitudinal::modeParamCallback(const ctrl_msgs::ModeParameters::ConstPtr& msg){
    double trq = msg->desiredMaximumTorque;
    double brake_trq = fabs(msg->desiredMaximumRegenTorque);
    if(this->TrqLimited && !this->TROflag && trq > 0.0 && brake_trq > 0.0){
        double ax = this->trq2acc(trq, this->m, this->radius);
        double ax_brake = this->trq2acc(brake_trq, this->m, this->radius);

        ROS_ERROR("LONG PID: Limiting ax: %f", ax);
        ROS_ERROR("LONG PID: Limiting ax brake: %f", ax_brake);

        if(0.0 < ax && ax < this->ax_acc_max) this->ax_acc_max = ax;  // we limit our longitudinal acceleration to fulfill the trq limitation
        if(0.0 < ax_brake && ax_brake < this->ax_dec_max) this->ax_dec_max = ax_brake;
        if(0.0 < ax && ax < this->ay_max) this->ay_max = ax; // safety condition (optional)

        this->update_kp(ax_acc_max, ax); // update Kp from PID

        ax_mode = ax;
        ax_dec_mode = ax_brake;

        ROS_ERROR("LONG PID: Limiting ax final: %f", ax_acc_max);
    }
    this->modeParamFlag = true;
}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Principal functions--------------------------------------------------

void Longitudinal::run(){

    // Compute index of velocity vector

        // We just computed a new velocity profile
    if(this->velFlag){

        latency = this->firstLatency;
        this->velFlag = false;

        // We just updated car's current progress
    }else if(this->posFlag){

        latency = max( min( (int) velocities.size()-1, int( (current_s-planner(0,2))/spacing ) ), this->firstLatency);
        this->posFlag = false;

        // We are going faster than our callbacks (so we predict the current progress of the car)
    }else if(this->firstvelFlag){

        // latency += max( min( (int) velocities.size()-1-latency, int(fabs(carState(3))*fabs(cos(carState(2)))*(1.0/freq)/spacing) ), this->firstLatency);
         /*NOTE: here |vx|*|cos(heading)| is done because we want the modulus (and we avoid negative values of vx due to variance around 0) */

    }else{
        return;
    }

    this->target_vel = max( min(velocities(latency), this->vx_max), this->vx_min ); // We set a maximum & minimum velocity

    // this->save<double>(this->savePath, "current_velocity", carState(3), true, false);
    // this->save<double>(this->savePath, "target_velocity", velocities(latency), true, false);

    // Apply PI:
    if(this->time2reset()) pid->reset_integral();
    this->throttle = pid->calculate_anti_windup(target_vel, carState(3));

    // this->save<double>(this->savePath, "tracking_error", target_vel-carState(3), true, false);

}

void Longitudinal::timeVelProf(){

    velocities.resize(planner.rows()-1);      // final velocity profile vector
    accelerations.resize(planner.rows()-1);   // final acceleration profile vector

    double ay = pow(carState(3), 2)*fabs(planner(0,3)); // current ay

        // First step: restrain limit acceleration
    Eigen::VectorXd vAccel(planner.rows()); 
    Eigen::VectorXd aAccel(planner.rows()-1); 

    vAccel(0) = fabs(carState(3)); // we start at current vx

    for(unsigned int j=0; j < planner.rows()-1; j++){

        aAccel(j) = (ay < ay_max) ? ax_acc_max * sqrt(lluisrho-pow(ay/ay_max, 2)) : 0.0; // available ax following GG diagram
        vAccel(j+1) = sqrt( 2*aAccel(j) * spacing + pow(vAccel(j), 2) ); // Vi+1² - Vi² = 2*ax*delta_x

        if( vAccel(j+1) > sqrt(ay_max/fabs(planner(j+1,3))) ){
            vAccel(j+1) = sqrt(ay_max/fabs(planner(j+1,3)));
            aAccel(j) = 0;
        }
        ay = pow(vAccel(j+1), 2)*fabs(planner(j+1, 3)); // ay prediction for next state
    }

        // Second step: restrain limit deceleration (from the end to the beginning)
    Eigen::VectorXd vDecel(planner.rows());
    Eigen::VectorXd aDecel(planner.rows()-1);

    vector<int> apexes = findLocalMax(planner.col(3)); // find local curvature maximums
    /* NOTE: The curvature apexes are the only points where we know the maximum velocity we could achieve (ay = v²/R) */

    // printVec(apexes,0);

            //CASE 1: We compute the velocity profile from the last apex to the end (actually the beginning)
    if(apexes.size() > 2 && lastApex){

        vDecel.resize(apexes[apexes.size()-1]+1);
        aDecel.resize(apexes[apexes.size()-1]);
        vDecel(0) = vAccel(apexes[apexes.size()-1]);

        int idx; // global index in planner matrix
        for(unsigned int j=0; j < apexes[apexes.size()-1]; j++){

            idx = apexes[apexes.size()-1]-j; // transform index to global (we are going backwards)
            ay = pow(vDecel(j), 2)*fabs(planner(idx, 3));

            aDecel(j) = (ay < ay_max) ? ax_dec_max * sqrt(lluisrho-pow(ay/ay_max, 2)) : 0.0; // available ax following GG diagram
            vDecel(j+1) = sqrt( 2*aDecel(j) * spacing + pow(vDecel(j), 2) ); // Vi+1² - Vi² = 2*ax*delta_x

            if( vDecel(j+1) > sqrt(ay_max/fabs(planner(idx-1,3))) ){
                vDecel(j+1) = sqrt(ay_max/fabs(planner(idx-1,3)));
                aDecel(j) = 0;
            }
        }

            //CASE 2: We compute the velocity profile from a fixed (or target) velocity to the end (beginning)
    }else{

        if(lastApex) ROS_WARN("Longitudinal: Couldn't use the last apex in vel. profile! There are not enough apexes");

        vDecel(0) = this->vx_final; // target final velocity

        int idx; // global index in planner matrix
        for(unsigned int j=0; j < planner.rows()-1; j++){

            idx = planner.rows()-1-j;   // transform index to global (we are going backwards)
            ay = pow(vDecel(j), 2)*fabs(planner(idx, 3));

            aDecel(j) = (ay < ay_max) ? ax_dec_max * sqrt(lluisrho-pow(ay/ay_max, 2)) : 0.0; // available ax following GG diagram
            vDecel(j+1) = sqrt( 2*aDecel(j) * spacing + pow(vDecel(j), 2) ); // Vi+1² - Vi² = 2a*delta_x

            if( vDecel(j+1) > sqrt(ay_max/fabs(planner(idx-1,3))) ){
                vDecel(j+1) = sqrt(ay_max/fabs(planner(idx-1,3)));
                aDecel(j) = 0;
            }
        }
    }
    Eigen::VectorXd vDecelGlobal = vDecel.reverse(); // reverse vDecel vector 
    Eigen::VectorXd aDecelGlobal = aDecel.reverse(); // reverse aDecel vector 

        // Third step: pick min values from both profiles
    for(unsigned int k=0; k < velocities.size(); k++){

            // From the last apex to the end we choose the accelerating profile
        if(apexes.size() > 2 && lastApex){
            if(k <= apexes[apexes.size()-1]){
                if(vAccel(k) < vDecelGlobal(k)){
                    velocities(k) = vAccel(k);
                    accelerations(k) = aAccel(k);
                }else{
                    velocities(k) = vDecelGlobal(k);
                    accelerations(k) = -aDecelGlobal(k);
                }
            }else{
                velocities(k) = vAccel(k); 
                accelerations(k) = aAccel(k);
            }

            // We pick a final velocity so we always choose the minimum velocity
        }else{
            if(vAccel(k) < vDecelGlobal(k)){
                velocities(k) = vAccel(k);
                accelerations(k) = aAccel(k);
            }else{
                velocities(k) = vDecelGlobal(k);
                accelerations(k) = -aDecelGlobal(k);
            }
        }
    }
    // saveEigen(this->savePath, "velocity_profile.csv", velocities, true);
    // saveEigen(this->savePath, "v_accel.csv", vAccel, true);
    // saveEigen(this->savePath, "v_decel.csv", vDecelGlobal, true);
    // saveEigen(this->savePath, "curvature.csv", planner.col(3), true);
    // saveEigen(this->savePath, "a_accel.csv", aAccel, true);
    // saveEigen(this->savePath, "a_decel.csv", aDecel, true);
    // saveEigen(this->savePath, "accelerations.csv", accelerations, true);
}

void Longitudinal::spatialVelProf(){

    velocities.resize(planner.rows()-1);      // final velocity profile vector
    accelerations.resize(planner.rows()-1);   // final acceleration profile vector

        // First step: restrain limit acceleration
    Eigen::VectorXd vAccel(planner.rows()); 
    Eigen::VectorXd aAccel(planner.rows()-1); 
    vAccel(0) = max(fabs(carState(3)), this->vx_min);
    for(unsigned int j=0; j < planner.rows()-1; j++){
        aAccel(j) = f_accel(j, vAccel(j));
        vAccel(j+1) = vAccel(j) + spacing*aAccel(j); // Euler integration
        if( vAccel(j+1) > sqrt(ay_max/fabs(planner(j+1,3))) ){
            vAccel(j+1) = sqrt(ay_max/fabs(planner(j+1,3)));
            aAccel(j) = 0;
        }
    }

        // Second step: restrain limit deceleration (from the end to the beggining)
    Eigen::VectorXd vDecel(planner.rows());
    Eigen::VectorXd aDecel(planner.rows()-1);
    vector<int> apexes = findLocalMax(planner.col(3));
    // printVec(apexes,0);

    if(apexes.size() > 2 && lastApex){
        vDecel.resize(apexes[apexes.size()-1]+1);
        aDecel.resize(apexes[apexes.size()-1]);
        vDecel(0) = vAccel(apexes[apexes.size()-1]); 
        int idx; // global index in planner matrix
        for(unsigned int j=0; j < apexes[apexes.size()-1]; j++){
            idx = apexes[apexes.size()-1]-j;   // transform index to global (we are going backwards)
            aDecel(j) = f_decel(idx, vDecel(j));
            vDecel(j+1) = vDecel(j) + spacing*aDecel(j);  // Euler integration
            if( vDecel(j+1) > sqrt(ay_max/fabs(planner(idx-1,3))) ){
                vDecel(j+1) = sqrt(ay_max/fabs(planner(idx-1,3)));
                aDecel(j) = 0;
            }
        }
    }else{
        if(lastApex) ROS_WARN("Longitudinal: Couldn't use the last apex in vel. profile! There are not enough apexes");
        vDecel(0) = this->vx_final;
        int idx; // global index in planner matrix
        for(unsigned int j=0; j < planner.rows()-1; j++){
            idx = planner.rows()-1-j;   // transform index to global (we are going backwards)
            aDecel(j) = f_decel(idx, vDecel(j));
            vDecel(j+1) = vDecel(j) + spacing*aDecel(j); // Euler integration
            if( vDecel(j+1) > sqrt(ay_max/fabs(planner(idx-1,3))) ){
                vDecel(j+1) = sqrt(ay_max/fabs(planner(idx-1,3)));
                aDecel(j) = 0;
            }
        }
    }
    Eigen::VectorXd vDecelGlobal = vDecel.reverse(); // reverse vDecel vector 
    Eigen::VectorXd aDecelGlobal = aDecel.reverse(); // reverse aDecel vector 

        // Third step: pick min values from both profiles
    for(unsigned int k=0; k < velocities.size(); k++){

            // from the last apex to the end we choose the accelerating profile
        if(apexes.size() > 2 && lastApex){
            if(k <= apexes[apexes.size()-1]){
                if(vAccel(k) < vDecelGlobal(k)){
                    velocities(k) = vAccel(k);
                    accelerations(k) = aAccel(k);
                }else{
                    velocities(k) = vDecelGlobal(k);
                    accelerations(k) = -aDecelGlobal(k);
                }
            }else{
                velocities(k) = vAccel(k); 
                accelerations(k) = aAccel(k);
            }

            // we pick a final velocity
        }else{
            if(vAccel(k) < vDecelGlobal(k)){
                velocities(k) = vAccel(k);
                accelerations(k) = aAccel(k);
            }else{
                velocities(k) = vDecelGlobal(k);
                accelerations(k) = -aDecelGlobal(k);
            }
        }
    }

    // saveEigen(this->savePath, "velocity_profile.csv", velocities, true);
    // saveEigen(this->savePath, "v_accel.csv", vAccel, true);
    // saveEigen(this->savePath, "v_decel.csv", vDecelGlobal, true);
    // saveEigen(this->savePath, "curvature.csv", planner.col(3), true);
    // saveEigen(this->savePath, "a_accel.csv", aAccel, true);
    // saveEigen(this->savePath, "a_decel.csv", aDecel, true);
    // saveEigen(this->savePath, "accelerations.csv", accelerations, true);
}

bool Longitudinal::isFinish(){
    return this->finished;
}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Auxiliar functions--------------------------------------------------

void Longitudinal::msgCommands(as_msgs::CarCommands *msg){ // Prepare msg to send to CTRL master
    
    msg->header.stamp = ros::Time::now();
    msg->motor = throttle;

    ROS_ERROR("LAPCOUT: %i", lapcount->laps);

    if( (bool) (this->mission == 0 && lapcount->laps>=1) || (bool) (this->mission==1 && lapcount->laps>=10) || (bool) (this->mission>=2 && preFinish) ){
        msg->motor = -1.0; // braking!
        if(carState(3) <= this->minVelFinish) finished = true; // we can publish finish flag!
        ROS_ERROR("FINISH LINE CROSSED!!!");
    }

    // save<double>(this->savePath, "throttle.csv", throttle, true);

    return;
}

void Longitudinal::msgVelocity(as_msgs::CarVelocityArray *msg){ // Prepare msg to send to Tailored MPC

    msg->velocities.clear();
    msg->header.stamp = ros::Time::now();

    if(this->firstvelFlag){
        int idx = this->latency - this->firstLatency;
        as_msgs::CarVelocity vel_msg;
        for(int i=idx; i<this->velocities.size(); i++){
            vel_msg.x = min(this->velocities(i), this->vx_max);
            msg->velocities.push_back(vel_msg);
        }
    }
}

void Longitudinal::online_planning(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg){

    // int idx0 = first_index(msg);
    planner.resize(this->nPlanning, 4);
    for (unsigned int i = 0; i < this->nPlanning; i++)
    {	
        planner(i, 0) = msg->objectives[i].x;
        planner(i, 1) = msg->objectives[i].y;
        planner(i, 2) = msg->objectives[i].s; 
        planner(i, 3) = (msg->objectives[i].k == 0.0) ? 1e-7 : msg->objectives[i].k;
    }

}

void Longitudinal::offline_planning(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg){

    // int idx0 = first_index(msg);
    planner.resize(msg->objectives.size(), 4);
    velocities.resize(msg->objectives.size());
    for (unsigned int i = 0; i < msg->objectives.size(); i++)
    {	
        planner(i, 0) = msg->objectives[i].x;
        planner(i, 1) = msg->objectives[i].y;
        planner(i, 2) = msg->objectives[i].s; 
        planner(i, 3) = (msg->objectives[i].k == 0.0) ? 1e-7 : msg->objectives[i].k; // avoid absolut zeros
        velocities(i) = this->troProfileMU * msg->objectives[i].vx; 
    }

}

bool Longitudinal::time2replan(){ // Decide if we must recompute the velocity profile
    if(firstvelFlag){
        double dist = planner(planner.rows()-1, 2) - this->current_s;
        if(dist < 0) dist += smax; // we made another lap

        if( dist < this->minDist ){
            ROS_WARN("TIME TO REPLAN!!");
            return true; 
        }else return false;

    }else{
        return true; // we want to compute a first velocity profile!
    }
}

int Longitudinal::first_index(const as_msgs::ObjectiveArrayCurv::ConstPtr& msg){

	double dist = 0.0, minDist = 100.0;
    int firstIdx = 0;
	Eigen::Vector2d position, car_direction;

    if(this->stateFlag){
        car_direction(0) = cos(carState(2));
        car_direction(1) = sin(carState(2));
        for (unsigned int i = 0; i < this->nSearch; i++){

            position(0) = (msg->objectives[i].x - carState(0));
            position(1) = (msg->objectives[i].y - carState(1));

            Eigen::Vector2d position_norm_vec = position/position.norm();

            // Only take into account those points with an angle of <65º with respect to the actual position
            if ( position_norm_vec.dot(car_direction) > cos(65.0 / 180.0 * M_PI)) {

                dist = position.norm();
                if (dist < minDist){
                    minDist = dist;
                    firstIdx = i;
                }
            }
        }
    }

    return firstIdx;
}

double Longitudinal::vel2acc(double &vf, double &vi, int &lat, int &pre_lat, double &delta_x){
    if(lat == pre_lat) pre_lat = 0;
    return ( pow(vf, 2) - pow(vi, 2) ) / (2*delta_x*double(lat-pre_lat));
}

double Longitudinal::trq2acc(double &trq, double &mass, double &radius){
    return trq/(mass*radius);
}

// EDO models of the acceleration physics
double Longitudinal::f_accel(int k, double v){
    if(v < sqrt(ay_max/fabs(planner(k,3))) ){
        return ax_acc_max/v * sqrt( this->lluisrho - pow(pow(v,2)*fabs(planner(k,3))/ay_max, 2) ); // EDO Model
    } else {
        return 0; // Can't accelerate more
    }
}
double Longitudinal::f_decel(int k, double v){
    if(v < sqrt(ay_max/fabs(planner(k,3))) ){
        return ax_dec_max/v * sqrt( this->lluisrho - pow(pow(v,2)*fabs(planner(k,3))/ay_max, 2) ); // EDO Model
    } else {
        return 0; // Can't decelerate more
    }
}

template<typename mytype> 
void Longitudinal::printVec(vector<mytype> &input, int firstElements){
    if(firstElements!=0){
      for (auto it = input.begin(); it != input.end()-input.size()+firstElements; it++) {
        cout << *it << "\n";
      }
    }else{
        for (auto it = input.begin(); it != input.end(); it++) {
        cout << *it << "\n";
      }
    }   
}

// Whether its time to reset integral term from PID
bool Longitudinal::time2reset(){
    if(carState(3) < this->minVelFinish) return true;
    else return false;
}

// Updates Kp variable from pid obj
void Longitudinal::update_kp(double &ax, double &limit_ax){
    this->pid->Kp = this->Kp*(ax/limit_ax);
    ROS_ERROR("PID KP: %f", this->pid->Kp);
}

// Find curvature apexes
vector<int> Longitudinal::findLocalMax(Eigen::VectorXd curv){ 
    vector<int> result;

    if(curv(0) > curv(1)) result.push_back(0); // Check whether the first point is local max
    
    for(int i=1; i < curv.size()-1; i++){
        if(curv(i-1) < curv(i) && curv(i) > curv(i+1)) result.push_back(i);
    }

    if(curv(curv.size()-1) > curv(curv.size()-2)) result.push_back(curv.size()-1); // Check whether the last point is local max

    return result;
}

void Longitudinal::saveEigen(string filePath, string name, Eigen::MatrixXd data, bool erase){

    try{
        
        // Write csv
        ofstream file;
        if(erase) file.open(filePath + name, ios::out | ios::trunc);
        else file.open(filePath + name, ios::app);

        //https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
        const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");

        if (file.is_open()){
            file << data.format(CSVFormat);
            file.close(); // Remeber to close the file!
        }

        // ROS_INFO_STREAM("MPC: matrix data SAVED AT: " << this->savePath+name);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM( "Longitudinal::saveEigen Exception was thrown: " << e.what() );
    }
}

template<typename mytype>
void Longitudinal::save(string filePath, string name, mytype data, bool time, bool unique){

    try{
        
        // Write csv
        ofstream file;
        std::string full_name;
        if(unique){
            std::string datetime = currentDateTime();
            datetime = regex_replace(datetime, regex(":"), "-");
            full_name = filePath + name + "_" + datetime + ".csv";
        }else{
            full_name = filePath + name;
        }
        file.open(full_name, ios::app);
        if(time) file << ros::Time::now().toSec() << "," << data << endl;
        else file << data << endl;
        file.close(); // Remeber to close the file!

        // ROS_INFO_STREAM("MPC: data SAVED AT: " << this->savePath+name);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM( "MPC::save Exception was thrown: " << e.what() );
    }
}

const string Longitudinal::currentDateTime(){ // Get current date/time, format is YYYY-MM-DD.HH:mm:ss
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}

void Longitudinal::reconfigure(long_pid::longConfig& config){

    try{
        this->ax_acc_max = config.Ax_acc_max;
        this->ax_dec_max = config.Ax_dec_max;
        this->ay_max = config.Ay_max;
        this->vx_final = config.Vx_final;
        this->vx_max = config.Vx_max;
        this->vx_min = config.Vx_min;
        this->Kp = config.Kp; // here PID::Kp is not modified because this is meant to be done in Long::update_kp()
        this->pid->Ki = config.Ki;
        this->pid->Kd = config.Kd;
        this->troProfileMU = config.troProfileMU;
        this->lluisrho = config.lluisrho;
        this->lastApex = config.lastApex;
        this->minDist = config.minDist;
        this->spatialFlag = config.spatialFlag;
        this->firstLatency = config.latency;
        this->TrqLimited = config.TrqLimited;

        ROS_WARN_STREAM("Ki: " << pid->Ki);
        ROS_WARN_STREAM("Kp: " << pid->Kp);
        ROS_WARN_STREAM("Kd: " << pid->Kd);
        ROS_WARN_STREAM("ax_acc_max: " << ax_acc_max);
        ROS_WARN_STREAM("ax_dec_max: " << ax_dec_max);
        ROS_WARN_STREAM("ay_max: " << ay_max);
        ROS_WARN_STREAM("vx_min: " << vx_min);
        ROS_WARN_STREAM("vx_max: " << vx_max);
        ROS_WARN_STREAM("lluisrho: " << lluisrho);

        this->dynParamFlag = true;

    } catch (exception& e){
        ROS_ERROR_STREAM("Longitudinal::RECONFIGURE DIED" << e.what());
    }
}
