// Simple PID algorithm with a wheel and tachometer. Plots variables to the serial monitor. Adjust parameters and see how they affect the system response
// By Nick Farago
// Parsing the serial input code from https://www.baldengineer.com/arduino-multi-digit-integers.html


//Pin Varibles
int motorCW = 6;      // Motor pin CW
int motorCCW = 5;     // Motor pin CCW
int pwm = 9;          // PWM pin
int sensor = 3;       // Sensor pin

//State Varaibles
volatile unsigned long plotCount = 0; //counts at interupt
volatile unsigned long controlCount = 0; //counts at interupt
volatile unsigned long past = 0;

//PID Variables
long error = 0;
long errorPrevious = 0;
long errorSum = 0;
double power = 0;
volatile double omega = 0;

//Other Variables
unsigned long plotIteration = 0; //cycle that is plotted
unsigned long controlIteration = 0; //cycle calculated
volatile double plotOmegaAvg = 0;  //running average of omega that is used for plotting
volatile double controlOmegaAvg = 0;  //running average of omega that is used for the control algorithm
unsigned int integerValue=0;  // max value is 65535, for input
char incomingByte;            // for input


//Adjustable Parameter Variables /////////////////////////////////////////////////////////////////////////////////////
long setpoint = 1000;
double Kp = .5;
double Ki = 0.3;
unsigned long totalDuration = 20; //time in s before program will stop
unsigned long plotDuration = 50; //time in ms between plot points
unsigned long controlDuration = 10; //time in ms between calculations for control algorithm
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void setup() {
  pinMode(motorCW, OUTPUT);
  pinMode(motorCCW, OUTPUT);
  pinMode(pwm, OUTPUT);
  pinMode(sensor, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensor), change, RISING);
  Serial.begin(9600);

  initialize();
}


void loop() {
  checkStop();
  plot();
  setPower();
  runMotor();
  checkInput();
}




void plot() { //plots data at specified duration. Also looks to see if there is a new setpoint
  if (plotDuration*plotIteration <= millis()-1000){
    plotIteration = ++plotIteration;
    //omega = ((count*1000*60)/(40*duration));

    
    Serial.print(error*Kp);
    Serial.print("\t");
    Serial.print(errorSum*Ki);
    Serial.print("\t");
    Serial.print(power);
    Serial.print("\t");
    Serial.print(plotOmegaAvg);
    Serial.println();
    
    plotOmegaAvg = 0;
    plotCount = 0;

  } 
}

/////////////////////////////////////////////  MAIN PID CONTROL HERE  /////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setPower(){  //uses algorithm to set power level at specified intervals
  if (controlDuration*controlIteration <= millis()-1000){
    controlIteration = ++controlIteration;

    errorPrevious = error;
    error = setpoint - controlOmegaAvg;
    errorSum = errorSum + errorPrevious;          //sum is calculated per time it takes for the controller to cycle, it should scale with any changes to controlDuration
    power = Kp*error + Ki*errorSum;
   
    controlOmegaAvg = 0;
    controlCount = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void runMotor(){  // runs motor at specified power level
  if (power > 1000) {    //bounds on power level
    power = 1000;
  }
  if (power < -1000) {
    power = -1000;
  }
  if (power  >= 0){         //set direction based on power
      digitalWrite(motorCW, LOW);
      digitalWrite(motorCCW, HIGH);
  }
  if (power < 0){
      digitalWrite(motorCCW, LOW);
      digitalWrite(motorCW, HIGH);
  }
  double magnitude = 255*(power/1000);  //convert 0-1000 scale to 0-255
  analogWrite(pwm, magnitude); // analogWrite values from 0 to 255
}


void change(){  //interupt sub that calculates speed based on pulse time change
  omega = 1000000/(micros()-past);
  past = micros();
  plotCount = ++plotCount;
  plotOmegaAvg = (plotOmegaAvg*(plotCount-1)+omega)/plotCount;
  controlCount = ++controlCount;
  controlOmegaAvg = (controlOmegaAvg*(controlCount-1)+omega)/controlCount;
  
}

void checkStop (){ // checks if program has reached the end time and if so ends the program
  if ((totalDuration*1000) <= millis()){
    power = 0;
    runMotor();
    while(1){  
    }
  }
  
}

void checkInput () { //checks if there is an input to the serial monitor, if so then set the setpoint to the input NOTE: using this stops the plotter
    if (Serial.available() > 0) {   // something came across serial
    integerValue = 0;      // throw away previous integerValue
    while(1) {        // force into a loop until 'n' is received
      incomingByte = Serial.read();
      if (incomingByte == '\n') break;   // exit the while(1), we're done receiving
      if (incomingByte == -1) continue;  // if no characters are in the buffer read() returns -1
      integerValue *= 10;  // shift left 1 decimal place
      // convert ASCII to integer, add, and shift left 1 decimal place
      integerValue = ((incomingByte - 48) + integerValue);
    }
      setpoint = integerValue;
  }
}


void initialize(){    //rotates motor to an encoder edge so that speed can be properly calculated. Resets all variables affected by the change() subroutine
  int initialState = digitalRead(sensor);
  //Serial.print("Initializing... ");
  //delay(1000);
  while (digitalRead(sensor) == initialState){
    power = 50;
    runMotor();
  }
  power = 0;
  runMotor();
  //Serial.println("Done.");
  delay(1000);
  past = micros();
  plotOmegaAvg = 0;
  plotCount = 0;
  controlOmegaAvg = 0;
  controlCount = 0;
  
}
