/*
	MuseumKeepon controller: v 2.0. Uses servos for all motors.
	-----------------------------------------------------------
	Slave module to the ArduinoController python module

	Notes:
	------ 
	* using the software servo library to free up Timer 1 to use for timing Bop motions accurately 
		Con: need to call SoftwareServo::refresh() every 20ms to update servo positions


	Ahsan Nawroj
	Updated: 03/24/2014
*/

#include <Arduino.h>
#include <string.h>
#include <SoftwareServo.h> 
#include <avr/io.h>
#include <avr/interrupt.h>
//#include "museumKeepon.h"

// --------------------------------------------------------------------------------
// 
//  PRE-PROCESSOR DIRECTIVES, DEFINITIONS
//
// --------------------------------------------------------------------------------
#define SERVO_PIN(x) (x+2)	// all servo pins are connected from 2 onwards
#define ROLL 0
#define BOP 1
#define TILT 2
#define PAN 3
#define SINGLE_BOP_DELAY 600 // --CALIBRATE--
#define BOP_OFF_SPEED 90 // --CALIBRATE--
#define DEFAULT_BOP_SPEED 45 // --CALIBRATE--

// --------------------------------------------------------------------------------
// 
//  G L O B A L S
//
// --------------------------------------------------------------------------------
#define FEAR 1
#define HAPPY 2
#define SAD 3
#define SURPRISE 4
#define NEUTRAL 5

SoftwareServo keeponServos[4];		// 0 = roll, 1 = bop, 2 = tilt, 3 = pan
String commandString;				// input string from Python master
bool commandsAreRelative = true;	// OPTIONAL: relative vs. absolute commands
bool newCommandReceived = false;	// flag for command received

int flag=0;
int panCommand, rollCommand, 
	bopCommand, tiltCommand, emotionCommand;	// global containers for inputs
int servoAngles[4];					// OPTIONAL: grouped container for inputs

unsigned long bopRunTime = 0;
unsigned long bopStartTime = millis();
unsigned long progStartTime;
int done = 0;

// --------------------------------------------------------------------------------
// 
//  M A I N   D R I V E R   F U N C T I O N S
//
// --------------------------------------------------------------------------------
void setup () {	
	Serial.begin(9600);
	Serial.println("In setup after serial initialization ");  
	
	// INITIALIZE SERVOS
	// -----------------
	for(int nServo = 0;nServo < 4;nServo++)  {
		keeponServos[nServo].attach(SERVO_PIN(nServo));
		keeponServos[nServo].write(90); // default starting location
		delay_sft(500);
	}
        delay(1000);
	Serial.println("Initialized servos");  
	Serial.println("Completed setup"); 
        progStartTime = millis();
}


void loop () {
	// --- tests ---
	// demo1_potToServos(); // show by manual tweaking that motors work
	//testAllSweeps();
        //keeponServos[PAN].write(125); delay_sft(1);
	// while(1) demo2_bopWithTiltRoll();
	//demo3_pureBop();
        
	unsigned long currentTime = millis();	
	if (bopRunTime > 0 && (currentTime-bopStartTime) > bopRunTime){
		keeponServos[BOP].write(BOP_OFF_SPEED);
		bopRunTime = 0;
	}

	checkForCommands();
	if (newCommandReceived) {
                switch(emotionCommand) {
                  case NEUTRAL : 
		    panTo(panCommand);
		    rollTo(rollCommand);
		    tiltTo(tiltCommand);
                    break;
                  case HAPPY :
                    actionHappy(); break;
                  case SAD :
                    actionSad();  break;
                  case FEAR :
                    actionFear(); break;
                  case SURPRISE :
                    actionSurprise(); break;
                  default: 
                    panTo(panCommand);
		    rollTo(rollCommand);
		    tiltTo(tiltCommand);
                    break;
                }
                Serial.println ("received: " + commandString);		
                //rollBy(rollCommand);
		//tiltBy(tiltCommand);
		// bopCnt(bopCommand);	// this is a count of bops to perform
	}
        if((currentTime-progStartTime) > 20){
          progStartTime = millis();
          //SoftwareServo::refresh();
        }
}



// --------------------------------------------------------------------------------
// 
//  U T I L I T Y   F U N C T I O N S
//
// --------------------------------------------------------------------------------

// Wrapper function for delays in this sketch
// Delays need to be chopped up to accomodate regular calls to SoftwareServo::refresh()
void delay_sft (long t) {
	if (t < 20) {delay(t); SoftwareServo::refresh(); }
	long i = t / 15;
	for (int j=0; j < i; j++) { delay(15); 	SoftwareServo::refresh(); }
	delay(t % 15); SoftwareServo::refresh(); 
}
 
// ------------------------------------------------------------------------
// Command parsing from serial stream
// ------------------------------------------------------------------------
// 	Expected string input:
// 		GGEmpXtYrZbT
// 		GG = "ab" (absolute), "re" (relative)
//              Em = 0 - Neutral, 1 - Happy, 2 - Sad, 3 - Fear, 4 - Surprise
// 		pX = pan value X
// 		tY = roll value Y
// 		rZ = tilt value Z
// 		bT = bop value T (count of how many times to bop)
// 
// 	Absolute positions 0-150 
// 	Relative pan positions 90-180 for positive 0 to 90, 90-0 for negative 0 to 90
// 		sending 90 does no position change

void checkForCommands() {
  newCommandReceived = false;
  if (Serial.available() <= 0) return; // leave right away 

	  char newByte; 
	  commandString = "";	// forget last command string
	 while (Serial.available() > 0) {	// read new command in
            newByte = (char)Serial.read();
            if (newByte != '\0') {//break;
	      commandString += newByte; 
	      delay_sft(1);
            }
	}
        commandString += '\0';
	if (commandString == "" || commandString.length() < 10) return; // bad command

	// relative vs. absolute commands
	//if (commandString[0] == 'a' && commandString[1] == 'b') commandsAreRelative = false;
	//else if (commandString[0] == 'r' && commandString[1] == 'e') commandsAreRelative = true;

        //Serial.println ("received: " + commandString); // return read command	

        int i = 0;
        while (commandString[i] != '\0') {
                if( commandString[i] == 'e' ) {
                  int j = i + 1;
                  while( commandString[j] != 'p' ) {
                    j++;
                  }
                  emotionCommand = (commandString.substring(i+1,j)).toInt();
                }
                if ( commandString[i] == 'p' ) {
                    //Serial.println(1);
                    int j = i + 1;
                    while ( commandString[j] != 't' ) {
                        j++;
                    }
                    panCommand = (commandString.substring(i+1,j)).toInt();
                    //Serial.println(panCommand);
                }
                else if ( commandString[i] == 't' ) {
                    //Serial.println(2);
                    int j = i + 1;
                    while ( commandString[j] != 'r' ) {
                        j++;
                    }
                    tiltCommand = (commandString.substring(i+1,j)).toInt();
                    //Serial.println(tmp);
                } 
                else if ( commandString[i] == 'r' ) {
                    //Serial.println(3);
                    int j = i + 1;
                    while ( commandString[j] != 'b' ) {
                        j++;
                    }
                    rollCommand = (commandString.substring(i+1,j)).toInt();
                    //Serial.println(tmp);
                } 
                else if ( commandString[i] == 'b' ) {
                    //Serial.println(4);
                    int j = i + 1;
                    while ( commandString[j] != '\0' ) {
                        j++;
                    }
                    bopCommand = (commandString.substring(i+1,j)).toInt();
                    //Serial.println(tmp);
                }
            i++;
        } newCommandReceived = true; 

/*
	// read actual command vars in
	int i = 2, j=0, tmp; // read the command into variables
	while (commandString[i] != '\0') {
                if ( commandString[i] == 'p' ) {
                    Serial.println(1);
                }
                else if ( commandString[i] == 't' ) {
                    Serial.println(2);
                } 
                else if ( commandString[i] == 'r' ) {
                    Serial.println(3);
                } 
                else if ( commandString[i] == 'b' ) {
                    Serial.println(4);
                }
      
		if (commandString[i] < '0' || commandString[i] > '9') { // a letter  
			for (j=i+1;j<i+5;j++)   // find the number after this letter
				if (commandString[j]=='\0' || commandString[j]<'0' || commandString[j]>'9') 
					tmp = (commandString.substring(i+1,j)).toInt();

			switch (commandString[i]) {
			case 'p': panCommand = tmp; Serial.println("pan: "); Serial.println(tmp); break;
			case 't': tiltCommand = tmp; Serial.println("tilt: "); Serial.println(tmp); break;
			case 'r': rollCommand = tmp; Serial.println("roll: "); Serial.println(tmp); break;
			case 'b': bopCommand = tmp; break;
			} 
			i = j;  // read next letter
		} i++;
	} newCommandReceived = true;	
*/
}

// ------------------------------------------------------
// Pan command angle between -90 (all the way left) and 90 (all the way right)
// ------------------------------------------------------
// relative pan
void panBy (int relativeAngle) {
	keeponServos[PAN].write(constrain(keeponServos[PAN].read() + relativeAngle, 20, 160));
}
// absolute pan
void panTo (int absoluteAngle) {
	keeponServos[PAN].write(constrain(absoluteAngle, 20, 160));
}

// ------------------------------------------------------
// Tilt command angle between -90 (all the way back) and 90 (all the way forward)
// ------------------------------------------------------
// relative tilt
void tiltBy (int relativeAngle) {
	keeponServos[TILT].write(constrain(keeponServos[TILT].read() + relativeAngle, 20, 160));
}
// absolute tilt
void tiltTo (int absoluteAngle) {
	keeponServos[TILT].write(constrain(absoluteAngle, 20, 160));
}

// ------------------------------------------------------
// Roll command angle is relative position from current. 
// ------------------------------------------------------
// relative roll
void rollBy (int relativeAngle) {
	keeponServos[ROLL].write(constrain(keeponServos[ROLL].read() + relativeAngle, 20, 160));
}
// absolute roll
void rollTo (int absoluteAngle) {
	keeponServos[ROLL].write(constrain(absoluteAngle, 20, 160));
}


// ------------------------------------------------------
// Bop command gives number of bops to make. 
// ------------------------------------------------------
void bopCnt (int cnt) {
	if (cnt == 0) return;

	keeponServos[BOP].write(DEFAULT_BOP_SPEED);
	long necessaryDelay = SINGLE_BOP_DELAY*cnt;	// in milliseconds
	bopStartTime = millis();
	bopRunTime = necessaryDelay;
}



// ------------------------------------------------------
// Freeze all motors 
// ------------------------------------------------------
void resetMotorPositions () {
	for(int nServo = 0;nServo < 4;nServo++)  {
		keeponServos[nServo].attach(SERVO_PIN(nServo));
		servoAngles[nServo] = 90; // default starting location
		delay_sft(1);
	}
}



// ------------------------------------------------------
//	Update motor locations
// ------------------------------------------------------
void motorStatusUp () {
	for(int nServo = 0;nServo < 4;nServo++)  {
		keeponServos[nServo].write(servoAngles[nServo]);
	}	
}

void printServoAngles() {
	for (int i=0;i<3;i++) {	Serial.print(servoAngles[i]); Serial.print(", "); }
	Serial.print(servoAngles[3]); Serial.println("");
}


// --------------------------------------------------------------------------------
// 
//  T E S T I N G   F U N C T I O N S
//
// --------------------------------------------------------------------------------
// read external pots, set servos to pot inputs
void demo1_potToServos () {
	pinMode(A0, INPUT);	pinMode(A1, INPUT); pinMode(A2, INPUT); pinMode(A3, INPUT);
	int potPins[4] = {A0, A1, A2, A3};
	int sensitivity[4] = {10, 10, 20, 20};
	while (1) {
		for (int i=0; i < 4; i++) {
			int j = analogRead(potPins[i]);
			j=(int) constrain(j*sensitivity[i] - 512*(sensitivity[i]-1), 0, 1023);
			servoAngles[i] = map(j, 0, 1023, 0, 180);
		}
		printServoAngles(); motorStatusUp(); delay_sft(10);
	}
}

// sweep tilt and roll back and forth, bop once every two seconds
void demo2_bopWithTiltRoll(){
	int testDelay = 5;
	for (int i = 20; i < 160; i++) { tiltTo(i); delay_sft(testDelay); }
	bopCnt(3);
	for (int i = 160; i > 20; i--) { tiltTo(i); delay_sft(testDelay); }
	delay_sft(3000);

	for (int i = 20; i < 160; i++) { rollTo(i); delay_sft(testDelay); }
	bopCnt(3);
	for (int i = 160; i > 20; i--) { rollTo(i); delay_sft(testDelay); }
	delay_sft(3000);
}

// simple bop test
// useful to calibrate bop timings
void demo3_pureBop () {
	bopCnt(3); delay_sft(5000); Serial.println("Bopped!");
}

// sweep through 0 to 180 for each servo
void testAllSweeps() {
	while(1) {
		int testDelay = 1;
		for (int i = 20; i < 160; i++) {
			keeponServos[PAN].write(i); delay_sft(testDelay);
		}	
		for (int i = 160; i > 20; i--) {
			keeponServos[PAN].write(i); delay_sft(testDelay);
		}
		
	}
}


void actionHappy () {
      for(int nServo = 0;nServo < 3;nServo++)  {
		keeponServos[nServo].write(90); // default starting location
      }
      delay_sft(500);
      keeponServos[ROLL].write(160);  delay_sft(300);
      keeponServos[ROLL].write(20);  delay_sft(300);
      keeponServos[ROLL].write(160);  delay_sft(300);
      keeponServos[ROLL].write(20);  delay_sft(300);
      keeponServos[ROLL].write(90); delay_sft(500); // default starting location
}


void actionSurprise () {
      for(int nServo = 0;nServo < 3;nServo++)  {
		keeponServos[nServo].write(90); // default starting location
      }
      delay_sft(500);
      if (flag==0)
      {
      keeponServos[TILT].write(160); keeponServos[BOP].write(160); delay_sft(500); delay(1000);
      keeponServos[BOP].write(90);
      delay_sft(300);
      flag=1;
      }
      
      else
      {
       keeponServos[TILT].write(160); keeponServos[BOP].write(20); delay_sft(500); delay(1000);
      keeponServos[BOP].write(90);
      delay_sft(300);
      flag=0;
      }
      //keeponServos[TILT].write(90); //keeponServos[BOP].write(20); 
      //delay_sft(500);
}



void actionFear() {
      for(int nServo = 0;nServo < 3;nServo++)  {
		keeponServos[nServo].write(90); // default starting location
      }
      delay_sft(500);  
          //keeponServos[BOP].write(160);
      //delay(400);
            keeponServos[TILT].write(160);
                  delay_sft(300);
         int i;         
                  for(i=0;i<5;i++)
                  {      keeponServos[ROLL].write(70);
                         delay_sft(100);
                         keeponServos[ROLL].write(110);
                         delay_sft(100);
                  
                  }
                  delay(800);
                  keeponServos[ROLL].write(90);
                  delay_sft(10);


     /* keeponServos[ROLL].write(160);
     // keeponServos[TILT].write(160);
    //  keeponServos[BOP].write(160);
      delay_sft(80);
      keeponServos[ROLL].write(20);
      //keeponServos[TILT].write(20);
     // keeponServos[BOP].write(20);
      delay_sft(80);     
      //keeponServos[TILT].write(160);
      //keeponServos[BOP].write(160);
      delay_sft(80);
      keeponServos[ROLL].write(20);
      //keeponServos[TILT].write(20);
      //keeponServos[BOP].write(20);
      delay_sft(80);
      keeponServos[ROLL].write(160);
      //keeponServos[TILT].write(160);
      //keeponServos[BOP].write(160);
      delay_sft(80);
      keeponServos[ROLL].write(20);
      //keeponServos[TILT].write(20);
      //keeponServos[BOP].write(20);
      delay_sft(80);     
      keeponServos[ROLL].write(160);
      //keeponServos[TILT].write(160);
      //keeponServos[BOP].write(160);
      delay_sft(80);
      keeponServos[ROLL].write(20);
      //keeponServos[TILT].write(20);
      //keeponServos[BOP].write(20);
            delay_sft(80); */
}

void actionSad () {
      for(int nServo = 0;nServo < 3;nServo++)  {
		keeponServos[nServo].write(90); // default starting location
      }
      delay_sft(500);
      keeponServos[TILT].write(90); keeponServos[BOP].write(90); delay_sft(20);

     // keeponServos[TILT].write(20); keeponServos[BOP].write(20); delay_sft(500); delay(1000);
      keeponServos[TILT].write(20); delay_sft(500); delay(1000);

      //keeponServos[TILT].write(90); keeponServos[BOP].write(160); delay_sft(500);
}

