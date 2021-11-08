#include "JeVois.h"

jevois::jevois(){
}

void jevois::init(){ 
    camera.Write(initChar, 7);
}

bool jevois::PathSelector(){
    for (int i = 0; i < 10; i++){
        Look();
        //get data from message
        x = std::strtol(m2.c_str(), &placeholder, 0);
        y = std::strtol(placeholder, &placeholder, 10);
        //compare values
        if (x < (Ax + xRange) && x > (Ax - xRange)){
            if (y < (Ay + yRange) && y > (Ay - yRange)){
                isA = true;
            }
        }
        //print values
        _sb.append("\traw: ");
        _sb.append(placeholder);
        _sb.append(" x: ");
	    _sb.append(std::to_string(x));
        _sb.append(" y: ");
	    _sb.append(std::to_string(y));
        _sb.append(" isA: ");
        _sb.append(std::to_string(isA));
		printf("%s\n",_sb.c_str());
        _sb.clear();
    }
    return isA;
}

void jevois::Look(){
    reading = true;
    for (int i = 0; i < 20; i++){
        message[i] = 'a';
    }
    m2 = "";
    index = 0;
    while (reading){
        camera.Read( protocol_buffer, 1);
        if (protocol_buffer[0] == '\n'){
            reading = false;
        }
        else {
            if (index > 2){
                m2.push_back(protocol_buffer[0]);
            }
            message[index] = protocol_buffer[0];
            index += 1;
        }
    }
    if (++_loops >= 50) {
		_loops = 0;
        _sb.append("\tcamera:");
	    _sb.append(message);

		  printf("%s\n",_sb.c_str());
      _sb.clear();
	}
}