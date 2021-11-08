#ifndef JEVOIS_H
#define JEVOIS_H
#include <cstring>
#include <string>
#include <iostream>
#include <frc/SerialPort.h>

class jevois{
    public:
        jevois();
        void init();
        void Look();
        bool PathSelector();

    private:
        std::string _sb;
        char protocol_buffer[1024] = {'a'};
        const char* initChar = "iseeyou";     
        int _loops = 0;
        bool reading = true;
        bool isA = false;
        char message[20] = {'a'};
        std::string m2;
        char * placeholder;
        long int x = 0;
        int Ax = 0;
        int xRange = 20;
        int y = 0;
        int Ay = 170;
        int yRange = 20;
        int index = 0;
        frc::SerialPort camera{115200, frc::SerialPort::Port::kUSB};
};
#endif
