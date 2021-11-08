#include "Drive.h"

Drive::Drive(){
}

void Drive::init(){
    
}

void Drive::Arcade(double x, double y){
    m_robotDrive.ArcadeDrive(x*0.8, y*0.6);
}