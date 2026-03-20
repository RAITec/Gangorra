#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define MILLISTOSECOND 1000

#define motorLeft 26
#define motorRight 25

//INTERVALO DE TEMPO ENTRE OS LOOP'S
int dt = 1;


//ACESSO A MPU / GYROSCOPIO
Adafruit_MPU6050 mpu;
bool calibration = false;

//VARIÁVEIS NECESSÁRIAS PARA UTILIZAR O GYROSCOPIO
float AngleRoll = 0, RateRoll = 0, RateCalibrationRoll = 0;
float KalmanAngleRoll  = 0, KalmanUncertaintyAngleRoll  = 2*2;
//KALMAN ANGLE ROLL É O ANGULO ATUAL

//CONFIGURAÇÃO DA MPU, NECESSÁRIO PARA UTILIZALA
//UTILIZE NA FUNÇÃO SETUP
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

//CALIBRAÇÃO PARA SETAR O ESTADO INICIAL DA MPU.
//DESCOBRIR O ANGULO INICIAL DA MPU
void CalibrarMPU()
{
 	for (int Passo = 0 ; Passo < 2000; Passo++) 
	{
		Serial.printf("Calibrando: %d\n", Passo);

		MPUgetSignalsLoop();

		RateCalibrationRoll  += RateRoll;
		delay(1);
	}

  calibration = true;
	
}


//FUNÇÃO COLOCADA NA LOOP, NECESSÁRIA PARA DESCOBRIR O ANGULO ATUAL DO GYROSCOPIO
//POR CONSEQUENCIA, DA GANGORRA/BALANÇA
void MPUgetSignalsLoop() 
{	
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float AceX = a.acceleration.x / 9.81; // m/s^2...
  float AceY = a.acceleration.y / 9.81; // ...
  float AceZ = a.acceleration.z / 9.81; // ...
 
  RateRoll  = g.gyro.x * 57.3; // °/s...

    
  AngleRoll  =  atan(AceY/sqrt(AceX*AceX + AceZ*AceZ))*1/(PI/180);
  
  if (calibration = true){
	RateRoll  -= (RateCalibrationRoll/2000)  - 1.2;
	AceX      -= 0.02;
	AceY      -= 0.03;
	AceZ      -= 0.13;
  }

  Kalman1D(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll); 
}


//FILTRO UTILIZADO PARA DESCOBRIR O ANGULO ATUAL COM POUCOS RUÍDOS
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





//FUNÇÃO NECESSÁRIA PARA USAR OS MOTORES
//DEVE SER USADA NA SETUP
void setupPWM(const int &freq, const int &resolution, const int &pin, const int &ch)
{
  pinMode(pin, OUTPUT); // Definição do pino de saída do PWM de controle do motor
  ledcSetup(ch, freq, resolution);// ...
  ledcAttachPin(pin, ch); // Funções para definição do PWM na ESP32
}

//SETA OS VALORES DE VELOCIDADE NOS MOTORES
void controlSpeed(int &speed, int ch)
{
	// Função para controlar a velocidade dos motores
	// min_sped=257 max_sped=511 para resolução = 10 bits
	ledcWrite(ch, speed); // Função para mudança do PWM na ESP32
}


//*****************************************************************************************************************
//
//  ESSE TRECHO DE CÓDIGO É NECESSÁRIO PARA FAZER O CONTROLE DA GANGORRA, ASSIM SENDO, HÁ OS TERMOS DE ERRO ATUAL 
//  E ANTERIOR, ASSIM COMO O ERRO INTEGRATIVO E VARIÁVEIS QUE SÃO NECESSÁRIAS PARA O CONTROLADOR.
//
//*****************************************************************************************************************

double timer;
bool stop = false;

double ref = 0;

double error = 0;
double prev_error = 0;

double integrative_error = 0;

// 1.2 // 0.0000000000001  //  0.000000000001

double K = 1;
double kP = 1.2;
double kI = 0.0000000000001;
double kD = 0.000000000001;

double gyro_const = 1;

//1000000

//LIMITES PARA OS PARAMETROS DE ENTRADA, POIS SE NÃO HOUVER, REFLETE MENOS O PARAMETROS FÍSICOS E PODE DANIFICAR
//OS DISPOSITIVOS
double Ulimit = 20;
double Ilimit = Ulimit;
double Dlimit = Ulimit;
double RPMlimit = 15;


//VARIÁVEIS PARA OS MOTORES E LIMITE DE USO
int motorLeftVel;
int motorRightVel;
int stopVel = 257;
int throttle = 275;


//CALCULO PARTE INTEGRATIVA DA ENTRADA
double getIntegrative_error(){
  integrative_error += kI * ((error + prev_error)*(dt)/2 * 1/MILLISTOSECOND);

  if(integrative_error > Ilimit) integrative_error = Ilimit;
  else if(integrative_error < -Ilimit) integrative_error = -Ilimit;

  return integrative_error;
}


//CACULO DA PARTE DERIVATIVA DA ENTRADA
double getDerivative_error(){
  double derivative = kD * (error - prev_error)/(dt);
  derivative *= MILLISTOSECOND;

  return derivative;
}



void pauseSystem(){
  if(Serial.available() > 0){
    stop = true;

    controlSpeed(stopVel,0);
    controlSpeed(stopVel,1);

    Serial.printf("Desliguei");
    delay(2000);
  }
}

void graphs(){
  Serial.print("Angulo:");
  Serial.print(KalmanAngleRoll);
  Serial.print(",");
  Serial.print("Velocidade Angular:");
  Serial.println(RateRoll);
}


void setup() {
  Serial.begin(115200);

  MPUconfigSetup(); //CONFIGURANDO A MPU
  CalibrarMPU();    //...

  controlSpeed(motorLeftVel, 0);    //CONFIGURANDO OS MOTORES
  controlSpeed(motorRightVel, 1);   //...

  Serial.printf("Começando...");    //...
  delay(2000);

  setupPWM(250, 10, motorLeft, 0);  //...
	setupPWM(250, 10, motorRight, 1); //...

  timer = millis();

  
  while(millis() - timer <= 2000){     //LOOPS NECESSÁRIOS PARA INICIALIZAR OS MOTORES
    controlSpeed(stopVel, 0);          //...
    controlSpeed(stopVel, 1);          //...
  }                                    //...
                                       //...
  timer = millis();                    //...
                                       //...
  while(millis() - timer <= 2000){     //...
    controlSpeed(throttle, 0);         //...
    controlSpeed(throttle, 1);         //...
  }                                    //...


}

void loop() {
  MPUgetSignalsLoop(); //FUNÇÃO PARA ATUALIZAR A POSIÇÃO ATUAL, DEVE SER UTILIZADA FORA DO LOOP DE CONTROLE
                       //PARA GARANTIR UMA ATUALIZAÇÃO DE POSIÇÃO ATUAL PRECISA

  if(millis() - timer >= dt){ //LOOP DE CONTROLE

  
    prev_error = error;
    error = ref - KalmanAngleRoll;
    
    double P = kP * error;
    double I = getIntegrative_error();
    double D = getDerivative_error();

    //CALCULO DE ENTRADA PARA O CONTROLADOR
    double U = P+I+D;

    U *= K;

    U = U - (gyro_const * RateRoll);

    //LIMITADOR DA ENTRADA, PARA IMPEDIR ENTRADAS QUE DANIFIQUEM O SISTEMA
    if(U > Ulimit) U = Ulimit;
    else if(U < -Ulimit) U = -Ulimit;

    double rpm;                               // TRANSFORMANDO A ENTRADA DO SISTEMA EM UM SINAL DE PWM,
    rpm = RPMlimit*((U + Ulimit)/Ulimit -1);  // FOI UTILIZADA UMA REGRA DE 3


    //MONITORAMENTO DO SINAL DE ENTRADA E PWM
    //Serial.printf("U: %f, RPM: %f \n" , U, rpm);

    //SETANDO OS VALORES DE PWM NOS MOTORES
    motorLeftVel = throttle - rpm;
    motorRightVel = throttle + rpm;

    //MONITORAMENTO DOS SINAL DE PWM NOS MOTORES
    //Serial.printf("Left: %d, Right: %d \n", motorLeftVel, motorRightVel);


    if(!stop){
      controlSpeed(motorLeftVel, 0);
      controlSpeed(motorRightVel, 1);
    }

    timer = millis();

    //MONITORAMENTO DO ANGULO E VELORICDADE ANGULAR
    //Serial.printf("Angle: %f, Rate: %f\n", KalmanAngleRoll, RateRoll);
  }


  graphs();

  //pauseSystem();

}
