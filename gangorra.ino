#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define MILLISTOSECOND 1000
#define MINTOSECONDS 60

#define motorLeft 26
#define motorRight 25
#define throttle 5

#define STEP_CALIBRATION 2000


int dt = 1;

Adafruit_MPU6050 mpu;
bool calibration = false;

float AngleRoll = 0, RateRoll = 0, RateCalibrationRoll = 0;
float KalmanAngleRoll  = 0, KalmanUncertaintyAngleRoll  = 2*2;



void MPUconfigSetup() 
{
	if (!mpu.begin()) 
	{	
		while (!mpu.begin()) 
		{
			Serial.println("error");
	  	yield();
		}
	}
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);//2G, 4G, 8G, 16G
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);     //250deg/s, 500deg/s, 1000deg/s, 2000deg/s
	mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);  //5Hz, 10Hz, 21Hz, 44Hz, 94Hz, 184Hz, 260Hz
}

void CalibrarMPU()
{
 	for (int Passo = 0 ; Passo < STEP_CALIBRATION; Passo++) 
	{
		Serial.printf("Calibrando: %d\n", Passo);

		MPUgetSignalsLoop();

		RateCalibrationRoll  += RateRoll;
		delay(1);
	}

  calibration = true;
	
}

void MPUgetSignalsLoop() 
{	
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float AceX = a.acceleration.x / 9.81; // m/s^2...
  float AceY = a.acceleration.y / 9.81; // ...
  float AceZ = a.acceleration.z / 9.81; // ...
 
  RateRoll  = g.gyro.x * 57.3; // °/s...

    
  AngleRoll  =  atan(AceY/sqrt(AceX*AceX + AceZ*AceZ))*1/(PI/180);
  
  if (calibration == true){
	RateRoll  -= (RateCalibrationRoll/STEP_CALIBRATION)  - 1.2;
	AceX      -= 0.02;
	AceY      -= 0.03;
	AceZ      -= 0.13;
  }

  Kalman1D(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll); 
}

void Kalman1D(float &KalmanState,float &KalmanUncertainty, const float &KalmanInput, 
					           const float &KalmanMeasurement)
{
	KalmanState       = KalmanState + 0.004*KalmanInput;
	KalmanUncertainty = KalmanUncertainty + 0.004*0.004*4*4;
	double KalmanGain = KalmanUncertainty*1/(1*KalmanUncertainty + 3*3);
	KalmanState       = KalmanState + KalmanGain*(KalmanMeasurement - KalmanState);
	KalmanUncertainty = (1 - KalmanGain)*KalmanUncertainty;
}


//********************************************************


int input_max = 2000;
int input_min = 1000;



void setupPWM(const int &freq, const int &resolution, const int &pin, const int &ch)
{
  pinMode(pin, OUTPUT); // Definição do pino de saída do PWM de controle do motor
  ledcSetup(ch, freq, resolution);// ...
  ledcAttachPin(pin, ch); // Funções para definição do PWM na ESP32
}

void controlSpeed(int &speed, int ch){
	// Função para controlar a velocidade dos motores
	// min_sped=257 max_sped=511 para resolução = 10 bits
	ledcWrite(ch, speed); // Função para mudança do PWM na ESP32
}




void readPWMSetup(const uint8_t &PinX){
	// Função para adequar os pinos que vão receber o PWM do controlador
	pinMode(PinX, INPUT);
}


int readPWMLoop(const uint8_t &PinX){
	return pulseIn(PinX, HIGH); 
}



//*****************************************************************************************************************

double timer;
bool stop = false;

double ref = 0;

double error = 0;
double prev_error = 0;

double integrative_error = 0;

double K = 1;
double kP = 2.5;
double kI = 0.89;
double kD = 0.0005;

double gyro_const = 1;

double Ulimit = 100;
double Ilimit = Ulimit;
double Dlimit = Ulimit;
double RPMlimit = 25;

int motorLeftVel;
int motorRightVel;
int stopVel = 257;
int throttleVel, throttleVel_back = 0;
int max_throttle = 400;


double getIntegrative_error(){
  integrative_error += kI * ((error + prev_error)*(dt)/2 * 1/MILLISTOSECOND);

  if(integrative_error > Ilimit) integrative_error = Ilimit;
  else if(integrative_error < -Ilimit) integrative_error = -Ilimit;

  return integrative_error;
}


double getDerivative_error(){
  double derivative = kD * (error - prev_error)/(dt);
  derivative *= MILLISTOSECOND;

  return derivative;
}



//*****************************************************************************************************************

int KvM = (1000 * 2* M_PI)/MINTOSECONDS;
double max_tension = 12;



void att_limits(){
  RPMlimit = throttleVel - stopVel;
  
  //Descobrir como alterar o U
}





void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  MPUconfigSetup();
  CalibrarMPU();


  controlSpeed(motorLeftVel, 0);
  controlSpeed(motorRightVel, 1);

  Serial.printf("Começando...");
  delay(2000);


  setupPWM(250, 10, motorLeft, 0);
	setupPWM(250, 10, motorRight, 1);

  //readPWMSetup(throttle);




  timer = millis();

  while(millis() - timer <= 2000){
    controlSpeed(stopVel, 0);
    controlSpeed(stopVel, 1);
  }

  timer = millis();

  while(millis() - timer <= 2000){
    controlSpeed(throttleVel, 0);
    controlSpeed(throttleVel, 1);
  }


}



void loop() {
  // put your main code here, to run repeatedly:
  MPUgetSignalsLoop();

  //Serial.printf("Angle: %f, Rate: %f\n", KalmanAngleRoll, RateRoll);

  
  if(millis() - timer >= dt){


    //throttleVel = readPWMLoop(throttle);
    throttleVel = 1150;

    

    throttleVel = ((throttleVel - input_min)*(max_throttle - stopVel))/(input_max - input_min) + stopVel;

    if(throttleVel_back == throttleVel){
      att_limits();
    }

    throttleVel_back = throttleVel;

    
    Serial.printf("Throttle: %d\n", throttleVel);

  
    prev_error = error;
    error = ref - KalmanAngleRoll;

    


    
    double P = kP * error;
    double I = getIntegrative_error();
    double D = getDerivative_error();

    
    double U = P+I+D;

    U *= K;

    U = U - (gyro_const * RateRoll);


    if(U > Ulimit) U = Ulimit;
    else if(U < -Ulimit) U = -Ulimit;

    double rpm;

    rpm = RPMlimit*((U + Ulimit)/Ulimit -1);


    Serial.printf("U: %f, RPM: %f RPMlimit: %f\n" , U, rpm, RPMlimit);

    motorLeftVel = throttleVel - rpm;
    motorRightVel = throttleVel + rpm;

    //motorLeftVel = 300;
    //motorRightVel = 300;

    

    Serial.printf("Left: %d, Right: %d \n", motorLeftVel, motorRightVel);


    if(!stop){
      controlSpeed(motorLeftVel, 0);
      controlSpeed(motorRightVel, 1);
      Serial.printf("Angle: %f, Rate: %f\n", KalmanAngleRoll, RateRoll);
    }

    timer = millis();


    
  }
  




  /*

  if(Serial.available() > 0){
    stop = true;

    controlSpeed(stopVel,0);
    controlSpeed(stopVel,1);

    Serial.printf("Desliguei");
    delay(2000);
  }
  */


}
