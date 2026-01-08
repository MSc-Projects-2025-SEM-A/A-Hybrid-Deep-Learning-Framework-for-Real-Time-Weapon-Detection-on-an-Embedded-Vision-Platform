### Note :i got this reference code from GitHub repository where I had used during the early development and testing phase



link:[link](https://github.com/antbern/gridmap-slam-robot/blob/master/robot/ARDUINO_SKETCH/ARDUINO_SKETCH.ino)



student:Aditya sattuluri

/\* This example shows how to get single-shot range

&nbsp;measurements from the VL53L0X. The sensor can optionally be

&nbsp;configured with different ranging profiles, as described in

&nbsp;the VL53L0X API user manual, to get better performance for

&nbsp;a certain application. This code is based on the four

&nbsp;"SingleRanging" examples in the VL53L0X API.



&nbsp;The range readings are in units of mm. \*/



\#include <Wire.h>

\#include <VL53L1X.h>

\#include <Servo.h>

\#include "PID\_v1.h"

\#include <math.h>



VL53L1X frontSensor;

VL53L1X backSensor;



\#define FRONT\_SENSOR\_SHUT A3

\#define BACK\_SENSOR\_SHUT A2



// stepper motor defines

\#define STEPPER\_EN A1

\#define STEPPER\_DIR 13

\#define STEPPER\_STEP 12

\#define STEPPER\_SENSOR A0



\#define MICROSTEPPING 2

\#define STEPS\_PER\_ROTATION (360\*MICROSTEPPING)



// motor defines

\#define MOTOR\_RIGHT\_EN 6

\#define MOTOR\_RIGHT\_DIRA 7

\#define MOTOR\_RIGHT\_DIRB 8

\#define MOTOR\_RIGHT\_ENCA 2 

\#define MOTOR\_RIGHT\_ENCB 4



\#define MOTOR\_LEFT\_EN 11

\#define MOTOR\_LEFT\_DIRA 9

\#define MOTOR\_LEFT\_DIRB 10

\#define MOTOR\_LEFT\_ENCA 3 

\#define MOTOR\_LEFT\_ENCB 5





// pin definitions using port and bit (FASTER :D)

\#define ENC\_PORT PIND

\#define LEFT\_ENCA\_BIT 3

\#define LEFT\_ENCB\_BIT 5



\#define RIGHT\_ENCA\_BIT 2

\#define RIGHT\_ENCB\_BIT 4



\#define ENC\_COUNTS\_PER\_REV (32 \* 30)



double h = 0;

double last\_timer\_us = 0;





// PID values for the motor controllers

double Kp = 0.55276367534483;//0.610694929511361;//0.641817786149385 ;//6.458906368104240;//5.061601496636267;//3.286079178973016;

double Ki = 1.64455966045303;//1.34329498731559;//1.169731890184110 ;//21.544597297186854;//59.064657944882540;//70.241507066863450; 

double Kd = 0.0101674410396297;//0.0220997968974464;

double Tf = 1/11.8209539589613;//1/5.57670843490099;





typedef struct {

&nbsp;   double Kp = 0.0;

&nbsp;   double Ki = 0.0;

&nbsp;   double Kd = 0.0;

&nbsp;   double Tf = 0.0;



&nbsp;   double P = 0.0;

&nbsp;   double I = 0.0;

&nbsp;   double D = 0.0;



&nbsp;   double e\_old = 0.0;

} PID\_t;



// struct defining the motor properties

typedef struct {

&nbsp;   int32\_t current\_encoder\_counter = 0;       // current encoder count

&nbsp;   int32\_t last\_encoder\_counter = 0;          // last encoder count, used for calculating rotational speed

&nbsp;   int32\_t odometry\_counter = 0;

&nbsp;	PID\_t pid;                                  // PID controller for velocity control

&nbsp;   double speed\_reference = 0;                 // the reference value used for the PID-controller    

&nbsp;   struct{

&nbsp;       char PIN\_EN, PIN\_DIRA, PIN\_DIRB;

&nbsp;   } pins;                                     // struct for the motors' pins

&nbsp;   

} motor\_t;



motor\_t motor\_left, motor\_right;





// sensor variables

unsigned short step\_counter = 0, last\_step\_counter = 0;



char doOnce = 0;

char doContinously = 0;



// the number of steps needed for a resolution of 2 degrees

short next\_steps = (short) (STEPS\_PER\_ROTATION \* (2.0f / 360.0f));



// Uncomment this line to use long range mode. This

// increases the sensitivity of the sensor and extends its

// potential range, but increases the likelihood of getting

// an inaccurate reading because of reflections from objects

// other than the intended target. It works best in dark

// conditions.



\#define LONG\_RANGE





// Uncomment ONE of these two lines to get

// - higher speed at the cost of lower accuracy OR

// - higher accuracy at the cost of lower speed





//#define HIGH\_SPEED

//#define HIGH\_ACCURACY







void setup(){

&nbsp;	Serial.begin(115200);

&nbsp;	Wire.begin();

&nbsp;	

&nbsp;	///////////// STEPPER MOTOR /////////////



&nbsp;	//Serial.print(left.control\_output);

&nbsp;	//Serial.println(left.pid->GetKp());



&nbsp;	// stepper motor:

&nbsp;	pinMode(STEPPER\_EN, OUTPUT);

&nbsp;	pinMode(STEPPER\_DIR, OUTPUT);

&nbsp;	pinMode(STEPPER\_STEP, OUTPUT);

&nbsp;	pinMode(STEPPER\_SENSOR, INPUT);

&nbsp;	

&nbsp;	// disable stepper motor

&nbsp;	digitalWrite(STEPPER\_EN, HIGH);

&nbsp;	

&nbsp;	// select rotational direction

&nbsp;	digitalWrite(STEPPER\_DIR, HIGH); 

&nbsp;	

&nbsp;	homeSensor();



&nbsp;	///////////// SENSORS ///////////////



&nbsp;	// disable both sensors first, theese are not level shifted, so only pull low when activated

&nbsp;	digitalWrite(FRONT\_SENSOR\_SHUT, LOW);

&nbsp;	digitalWrite(BACK\_SENSOR\_SHUT, LOW);



&nbsp;	pinMode(FRONT\_SENSOR\_SHUT, OUTPUT);

&nbsp;	pinMode(BACK\_SENSOR\_SHUT, OUTPUT);	



&nbsp;	// initialize front sensor

&nbsp;	pinMode(FRONT\_SENSOR\_SHUT, INPUT); // enable (high Z state)

&nbsp;	frontSensor.init();

&nbsp;	frontSensor.setTimeout(500);

&nbsp;	frontSensor.setAddress(0x29 + 0x02);

&nbsp; 

&nbsp;	pinMode(BACK\_SENSOR\_SHUT, INPUT); // enable (high Z state)

&nbsp;	backSensor.init();

&nbsp;	backSensor.setTimeout(500);

&nbsp;	backSensor.setAddress(0x29 + 0x04);



&nbsp;	/\*

\#if defined LONG\_RANGE

&nbsp;	// lower the return signal rate limit (default is 0.25 MCPS)

&nbsp;	frontSensor.setSignalRateLimit(0.1);

&nbsp;	backSensor.setSignalRateLimit(0.1);

&nbsp;	// increase laser pulse periods (defaults are 14 and 10 PCLKs)

&nbsp;	frontSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);

&nbsp;	backSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);



&nbsp;	frontSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

&nbsp;	backSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

\#endif



\#if defined HIGH\_SPEED

&nbsp;	// reduce timing budget to 20 ms (default is about 33 ms)

&nbsp;	frontSensor.setMeasurementTimingBudget(20000);

&nbsp;	backSensor.setMeasurementTimingBudget(20000);

\#elif defined HIGH\_ACCURACY

&nbsp;	// increase timing budget to 200 ms

&nbsp;	frontSensor.setMeasurementTimingBudget(200000);

&nbsp;	backSensor.setMeasurementTimingBudget(200000);

\#endif

&nbsp;	\*/



&nbsp;	frontSensor.setDistanceMode(VL53L1X::Long);

&nbsp;	backSensor.setDistanceMode(VL53L1X::Long);



&nbsp;	frontSensor.setMeasurementTimingBudget(20000);

&nbsp;	backSensor.setMeasurementTimingBudget(20000);



&nbsp;	// initialize with back-to-back mode

&nbsp;	frontSensor.startContinuous(20);

&nbsp;	backSensor.startContinuous(20);





&nbsp;	// sensors initialized, lets init the motors



&nbsp;	///////////// DC MOTORs ///////////

&nbsp;	pinMode(MOTOR\_LEFT\_EN, OUTPUT);	

&nbsp;   digitalWrite(MOTOR\_LEFT\_EN, HIGH);

&nbsp;   pinMode(MOTOR\_LEFT\_DIRA, OUTPUT);

&nbsp;   pinMode(MOTOR\_LEFT\_DIRB, OUTPUT);

&nbsp;   pinMode(MOTOR\_LEFT\_ENCA, INPUT);

&nbsp;   pinMode(MOTOR\_LEFT\_ENCB, INPUT);



&nbsp;   pinMode(MOTOR\_RIGHT\_EN, OUTPUT);

&nbsp;   digitalWrite(MOTOR\_RIGHT\_EN, HIGH);

&nbsp;   pinMode(MOTOR\_RIGHT\_DIRA, OUTPUT);

&nbsp;   pinMode(MOTOR\_RIGHT\_DIRB, OUTPUT);

&nbsp;   pinMode(MOTOR\_RIGHT\_ENCA, INPUT);

&nbsp;   pinMode(MOTOR\_RIGHT\_ENCB, INPUT);



&nbsp;   attachInterrupt(digitalPinToInterrupt(MOTOR\_LEFT\_ENCA), int\_encoder\_left, CHANGE);

&nbsp;   attachInterrupt(digitalPinToInterrupt(MOTOR\_RIGHT\_ENCA), int\_encoder\_right, CHANGE);





&nbsp;	// set up motors and their PID-regulators

&nbsp;   motor\_left.pid.Kp = 0.55276367534483;

&nbsp;   motor\_left.pid.Ki = 1.64455966045303;

&nbsp;   motor\_left.pid.Kd = 0.0101674410396297;

&nbsp;   motor\_left.pid.Tf = 1/11.8209539589613;

&nbsp;   motor\_left.pins = {MOTOR\_LEFT\_EN, MOTOR\_LEFT\_DIRA, MOTOR\_LEFT\_DIRB};



&nbsp;   motor\_right.pid.Kp = 0.55276367534483;

&nbsp;   motor\_right.pid.Ki = 1.64455966045303;

&nbsp;   motor\_right.pid.Kd = 0.0101674410396297;

&nbsp;   motor\_right.pid.Tf = 1/11.8209539589613;

&nbsp;   motor\_right.pins = {MOTOR\_RIGHT\_EN, MOTOR\_RIGHT\_DIRA, MOTOR\_RIGHT\_DIRB};

&nbsp;   	

&nbsp;	motor\_right.speed\_reference = 0.0f;

&nbsp;	motor\_left.speed\_reference = 0.0f;

&nbsp;	

&nbsp;	// initialize the time counter

&nbsp;	last\_timer\_us = micros();	

}



void homeSensor(){

&nbsp;	// enable stepper

&nbsp;	digitalWrite(STEPPER\_EN, LOW);

&nbsp;	

&nbsp;	int smoothed = 0, smoothed16 = 0, newVal;

&nbsp;	

&nbsp;	// startup value

&nbsp;	smoothed = analogRead(STEPPER\_SENSOR);

&nbsp;	smoothed16 = smoothed << 4;

&nbsp;	do{

&nbsp;		// do some exponential filtering. See https://forum.arduino.cc/index.php?topic=445844.0 (last post)

&nbsp;		smoothed16 = smoothed16 - smoothed + analogRead(STEPPER\_SENSOR);

&nbsp;		smoothed = smoothed16 >> 4;

&nbsp;		

&nbsp;		

&nbsp;		// step the motor	

&nbsp;		step\_motor(1);

&nbsp;		

&nbsp;		// get a new reading 

&nbsp;		newVal = analogRead(STEPPER\_SENSOR);



&nbsp;		// continue while we see no "peak" in the sensor value

&nbsp;	}while(newVal - smoothed < 25);

&nbsp;	

&nbsp;	// disable stepper

&nbsp;	digitalWrite(STEPPER\_EN, HIGH);

&nbsp;	

&nbsp;	// reset position step counter

&nbsp;	step\_counter = 0;

}





//unsigned short frontAngle = 0, backAngle = 0;

void loop() {

&nbsp;	// get current time

&nbsp;   unsigned long timer\_us = micros();



&nbsp;   // calculate elapsed time in seconds 

&nbsp;   h = (double)(timer\_us - last\_timer\_us) / 1000000.0; 



&nbsp;   // store current time for next iteration

&nbsp;   last\_timer\_us = timer\_us; 



&nbsp;	// handle motors

&nbsp;   handle\_motor(\&motor\_left, h);

&nbsp;   handle\_motor(\&motor\_right, h);





&nbsp;	// check if there are any incoming bytes on the serial port

&nbsp;	if(Serial.available() > 0){

&nbsp;		// read one byte

&nbsp;		char input = Serial.read();

&nbsp;		

&nbsp;		if(input == 0x01 || input == 'O'){ // "do once"-command?

&nbsp;			doOnce = 1;

&nbsp;			digitalWrite(STEPPER\_EN, LOW);

&nbsp;		}else if (input == 0x02 || input == 'E'){ // "enable continous"-command?

&nbsp;			doContinously = 1;

&nbsp;			doOnce = 1;

&nbsp;			digitalWrite(STEPPER\_EN, LOW);

&nbsp;		}else if (input == 0x04 || input == 'D'){ // "disable continous"-command?

&nbsp;			doContinously = 0;

&nbsp;		}else if (input == 0x05 || input == 'H'){ // "disable continous"-command?

&nbsp;			homeSensor();

&nbsp;		}else if (input == 0x08){ // "set resolution"-command?

&nbsp;			char d;

&nbsp;			// wait for next byte

&nbsp;			while ((d = Serial.read()) == -1);

&nbsp;			

&nbsp;			next\_steps = (short) (STEPS\_PER\_ROTATION \* ((float)d / 360.0f));

&nbsp;			

&nbsp;		}else if (input == 0x10){ // "set left motor speed" - command?

&nbsp;			// wait for 4 bytes

&nbsp;			while(Serial.available() < 4);

&nbsp;			

&nbsp;			// get the four bytes and convert them to a float

&nbsp;			char buffer\[4];

&nbsp;			buffer\[3] = Serial.read();

&nbsp;			buffer\[2] = Serial.read();

&nbsp;			buffer\[1] = Serial.read();

&nbsp;			buffer\[0] = Serial.read();

&nbsp;			

&nbsp;			float value = \*((float\*)\&buffer);

&nbsp;			

&nbsp;			motor\_left.speed\_reference = (double) value;

&nbsp;		}else if (input == 0x11){ // "set right motor speed" - command?

&nbsp;			// wait for 4 bytes

&nbsp;			while(Serial.available() < 4);

&nbsp;			

&nbsp;			// get the four bytes and convert them to a float

&nbsp;			char buffer\[4];

&nbsp;			buffer\[3] = Serial.read();

&nbsp;			buffer\[2] = Serial.read();

&nbsp;			buffer\[1] = Serial.read();

&nbsp;			buffer\[0] = Serial.read();

&nbsp;			

&nbsp;			float value = \*((float\*)\&buffer);

&nbsp;			motor\_right.speed\_reference = (double) value;

&nbsp;		}

&nbsp;		

&nbsp;	}



&nbsp;

&nbsp;	

&nbsp;	// only do measurement if 

&nbsp;	if(doOnce != 0){

&nbsp;				

&nbsp;		// move the motor

&nbsp;		step\_motor(next\_steps);

&nbsp;		

&nbsp;		// store the number of steps before increasing

&nbsp;		last\_step\_counter = step\_counter;

&nbsp;		

&nbsp;		// increase the counter and "loop back" if we exceed the number of steps for one compleete sensor revolution

&nbsp;		step\_counter = (step\_counter + next\_steps) % STEPS\_PER\_ROTATION;

&nbsp;		

&nbsp;		

&nbsp;		// check if we crossed any halfway

&nbsp;		if(last\_step\_counter % (STEPS\_PER\_ROTATION / 2) > (last\_step\_counter + next\_steps) % (STEPS\_PER\_ROTATION / 2)){

&nbsp;			// yes, send message (with odometry information) to indicate that

&nbsp;			sendData(-1, motor\_left.odometry\_counter, motor\_right.odometry\_counter);

&nbsp;			

&nbsp;			// reset odometry counters

&nbsp;			motor\_left.odometry\_counter = 0;

&nbsp;			motor\_right.odometry\_counter = 0;

&nbsp;			

&nbsp;			// this allows the code to either do repeated measurements (if doContinously = 1) or only do a measurement once (if doContinously = 0 and doOnce is set to 1 once)

&nbsp;			doOnce = doContinously;

&nbsp;			

&nbsp;			// if stopped, disable the motor

&nbsp;			if(doOnce == 0)

&nbsp;				digitalWrite(STEPPER\_EN, HIGH);

&nbsp;		}

&nbsp;		



&nbsp;		// take readings

&nbsp;		short frontRange = frontSensor.read();

&nbsp;		short backRange = backSensor.read();



&nbsp;		// check if max-range was recieved

&nbsp;		if(frontSensor.ranging\_data.range\_status == VL53L1X::RangeStatus::SignalFail || frontSensor.ranging\_data.range\_status == VL53L1X::RangeStatus::OutOfBoundsFail){

&nbsp;			frontRange = -1;

&nbsp;		}



&nbsp;		if(backSensor.ranging\_data.range\_status == VL53L1X::RangeStatus::SignalFail || backSensor.ranging\_data.range\_status == VL53L1X::RangeStatus::OutOfBoundsFail){

&nbsp;			backRange = -1;

&nbsp;		}



&nbsp;		// take and send the actual measurements

&nbsp;		sendData(step\_counter, frontRange, backRange);

&nbsp;	

&nbsp;	} else {

&nbsp;		delay(10);

&nbsp;	}



}







void step\_motor(unsigned short steps){

&nbsp;	for(unsigned short i = 0; i < steps; i++){

&nbsp;		digitalWrite(STEPPER\_STEP, HIGH);

&nbsp;		delayMicroseconds(1000);

&nbsp;		digitalWrite(STEPPER\_STEP, LOW);

&nbsp;		delayMicroseconds(1000);

&nbsp;	}

}



inline void sendData(short steps, short frontDistance, short backDistance){

&nbsp;	Serial.write(0x55); // start byte

&nbsp;	Serial.write((steps >> 8) \& 0xff);

&nbsp;	Serial.write((steps >> 0) \& 0xff); 

&nbsp;	Serial.write((frontDistance >> 8) \& 0xff);

&nbsp;	Serial.write((frontDistance >> 0) \& 0xff);

&nbsp;	Serial.write((backDistance >> 8) \& 0xff);

&nbsp;	Serial.write((backDistance >> 0) \& 0xff);

&nbsp;

}





void handle\_motor(motor\_t\* motor, double h){

&nbsp;   double speed = getMotorRotationSpeed(motor, h);



&nbsp;   // calculate error

&nbsp;   double e = motor->speed\_reference - speed;



&nbsp;   // calculate control signal

&nbsp;   double u = calculate\_pid(\&motor->pid, e, h);  



&nbsp;   // constrain control signal to within +-12 volts

&nbsp;   double saturated\_u = constrain(u, -12.0, 12.0);

&nbsp;   

&nbsp;   // drive the motor with this signal

&nbsp;   actuate\_motor(motor, saturated\_u); 



}



// does the PID calculation and returns the new control output

double calculate\_pid(PID\_t\* pid, double error, double h){

&nbsp;   // proportional part

&nbsp;   pid->P = pid->Kp \* error;



&nbsp;   // derivative part

&nbsp;   pid->D = pid->Tf / (pid->Tf + h) \* pid->D + pid->Kd / (pid->Tf + h) \* (error - pid->e\_old);



&nbsp;   // calculate output

&nbsp;   double u = pid->P + pid->I + pid->D;



&nbsp;   // integral part

&nbsp;   pid->I += pid->Ki \* h \* error;



&nbsp;   // save error

&nbsp;   pid->e\_old = error;



&nbsp;   // return control signal

&nbsp;   return u;

}



// motor function, input voltage in range -12 to 12 volts

void actuate\_motor(motor\_t\* motor, double u){

&nbsp;   // cap u in the range -12 to 12 volts

&nbsp;   u = constrain(u, -12.0, 12.0);





&nbsp;	// theese small voltages will only make the motors whine anyway

&nbsp;	if( abs(u) < 0.6)

&nbsp;		u = 0;



&nbsp;   // convert voltage to pwm duty cycle

&nbsp;   u = 100.0 \* (u / 12.0);



&nbsp;   // convert pwm duty cycle to raw value

&nbsp;   uint8\_t PWM\_VALUE = (uint8\_t) abs(u \* (double) 255 / 100.0 );



&nbsp;   analogWrite(motor->pins.PIN\_EN, PWM\_VALUE);

&nbsp;   if(u >= 0){

&nbsp;       // forward

&nbsp;       digitalWrite(motor->pins.PIN\_DIRA, HIGH);

&nbsp;       digitalWrite(motor->pins.PIN\_DIRB, LOW);

&nbsp;   }else{

&nbsp;       // backward

&nbsp;       digitalWrite(motor->pins.PIN\_DIRA, LOW);

&nbsp;       digitalWrite(motor->pins.PIN\_DIRB, HIGH);

&nbsp;   }

}


