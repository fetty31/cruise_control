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

#ifndef __LAPCOUNT_HH__
#define __LAPCOUNT_HH__

#include <cmath>
#include <eigen3/Eigen/Dense>

class Lapcount{

    public:
        // Lapcount functions & variables
        bool Lapchanged = true;  // Lapcount flag

        Eigen::Vector3d old_position   = Eigen::Vector3d::Zero(); // Last position
        Eigen::Vector3d position       = Eigen::Vector3d::Zero(); // Current position
        Eigen::Vector2d first_position = Eigen::Vector2d::Zero(); // Current position

        float m, n;                 // Starting line slope & intercept
        float distMax = 3;         // Lapcount threshold distance

        int laps = 0;               // Number of laps

        // Returns position relative to start/finish line
        float relativePosition(const Eigen::Vector3d point){
            return point(1)-m*point(0)-n;
        }

        // Get the new position and store the last one
        void setPosition(const Eigen::Vector3d point){
            old_position = position;
            position = point;
        }

        // Sets start/finish line
        void getStartingLine(){
            float angle = position(2)+M_PI/2.0;

            m = tan(angle);
            n = position(1)-m*position(0);

            // save initial position
            this->first_position(0) = position(0);
            this->first_position(1) = position(1);
        }

        void run(){

            double dist = sqrt(pow(position(0)-first_position(0),2)+pow(position(1)-first_position(1),2)); // Dist from the origin (0,0)

            // If we are close to the starting line (after having been far) and we cross the line, we have completed a lap
            if(dist <= distMax && !Lapchanged){

                if(relativePosition(old_position)*relativePosition(position)<=0){
                    laps++;
                    std::cout << "Hurray we made another lap!! " << laps << std::endl;
                    Lapchanged = !Lapchanged;
                }

            // We indicate that we have been far from the origin 
            }else if(dist > distMax){
                Lapchanged = false;
            }
        }
};

#endif