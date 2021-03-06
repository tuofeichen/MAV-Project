#ifndef PID_H_
#define PID_H_


class PID{

private:

float Kp;
float Ki;
float Kd;
int dt; // time difference 

float input;
float output;

float error;
float integral;
float errorPast;
float derivate;
float sensor;

////////////////////////////////////////////////////////////

public:

PID();

void setKp(float Kp);
void setKi(float Ki);
void setKd(float Kd);
void setInput(float input1);
void setSensor(float sensor1);
float update();

float getError();
float getIntegral();
float getErrorPast();
float getDerivate();
float getOutput();

};

//////////////////////////////////////////////////////
PID::PID(){
	Kp = 0;
	Ki = 0;
	Kd = 0;
	dt = 1;
	input = 0;
	output = 0;
	error=0;
    integral=0;
    errorPast=0;
    derivate=0;

    sensor=0;
}
/*
PID::PID(float Kp1,float Ki1, float Kd1, float input1){
	Kp = Kp1;
	Ki = Ki1;
	Kd = Kd1;
	input = input1;
	output = 0;
	error=0;
    integral=0;
    errorPast=0;
    derivate=0;

    sensor=0;
}*/


void PID::setKp(float Kp1){
	Kp = Kp1;
}

void PID::setKi(float Ki1){
	Ki = Ki1;
}

void PID::setKd(float Kd1){
	Kd = Kd1;
}

void PID::setInput(float input1){
	input = input1;
}

void PID::setSensor(float sensor1){
    sensor = sensor1;
}
////////////////////////////////////////////////////////////
float PID::update(){
    error = input - sensor;
	integral = integral + error;

    if ((error - errorPast) != 0){
		derivate = (error - errorPast)/(float)dt;
        	errorPast = error;
		dt = 1; 
    }
    else 
    {
    	dt++;
    }

    output = Kp*error + Ki*integral + Kd*derivate;
    return output;
}
/////////////////////////////////////////////////////////////
float PID::getError(){
	return error;
}

float PID::getIntegral(){
	return integral;
}

float PID::getErrorPast(){
	return errorPast;
}


float PID::getDerivate(){
	return derivate;
}


float PID::getOutput(){
	return output;
}


#endif
