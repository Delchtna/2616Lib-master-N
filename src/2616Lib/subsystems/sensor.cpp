#include "main.h"
#include "pros/misc.h"

double get_perp_dist(){
    double distance = (perp_sensor.get()/25.4);
    distance+=5.5;
    if(distance >72){
        distance -=72;
    }else if(distance < 72)
        distance = 72-distance;
    else if(distance == 72){
        return 0;
    }
    if((chassis.get_pose().angle)<.01||(90 - chassis.get_pose().angle)<.01){
            distance *= -1;
    }
    
    return distance;
}