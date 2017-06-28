/////////////////////////////////////////////////////
//
//      
//    
/////////////////////////////////////////////////////

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Kalman.h"

// PINES:
//0 y 1 para tx y rx del bluetooth
// 2 pin de interrupcion para el mpu 
#define P_ECHO_D            3     //Pin 03 como (ECHO PWM ) para el Ultrasonido Delantero
#define P_TRIGGER_D         4     //Pin 04 como (TRIGGER ) para el Ultrasonido Delantero
#define MOTOR_DIR           5     //Pin 05 Motor hacia izquierda
#define MOTOR_DIR_R         6     //Pin 06 Motor hacia derecha
#define LED_MARCHA_ADELANTE 7     //Pin 07 LED de adelante
#define LED_MARCHA_ATRAS    8     //Pin 08 LED de atras
#define MOTOR_TRACCION      9     //Pin 09 Motor hacia adelante
#define MOTOR_TRACCION_R    10    //Pin 10 Motor hacia atras
#define P_TRIGGER_A         11    //Pin 12 como (P_TRIGGER_) para el Ultrasonido Trasero
#define P_ECHO_A            12    //Pin 11 como (P_ECHO PWM ) para el Ultrasonido Trasero
#define LED_PIN             13    //Pin Led Arduino

//CONSTANTES
#define DISTANCIA_FRENO 50  //Para Sensores
#define TIEMPO_GIRO 200   //Para que GIRE durante 2 seg.
#define TIEMPO_FRENADO 1000
#define ADELANTE 'w'
#define REVERSA 's'
#define PARAR 'p'
#define DOBLAR_DERECHA 'd'
#define DOBLAR_IZQUIERDA 'a'
#define APAGAR 'o'
#define ENCENDER 'f'
#define LENTO 'h'
#define NORMAL 'j'
#define RAPIDO 'k'
#define MAX_VELOCIDAD 'l'

//Velocidades 
#define VEL_LENTO 64
#define VEL_NORMAL 128
#define VEL_RAPIDO 192
#define VEL_MAX 255   
MPU6050 mpu;

unsigned int velocidad = VEL_NORMAL;    // Velocidad de los motores (MACROS)
char estado = APAGAR;                   // Inicia detenido
long duracion, distanciaD, distanciaT;  // Para Calcular distacia

//Booleanos de estados
boolean obstaculo_D;                    // Determina si hay obstaculo DELANTERO
boolean obstaculo_T;                    // Determina si hay obstaculo TRASERO
boolean encendido=false; 
boolean hacia_delante=false;
boolean reversa=false;

//Kalman German
/*
x = x
p = p + q;
k = p / (p + r);
x = x + k * (measurement – x);
p = (1 – k) * p;

Donde:
q = covariancia del ruido del proceso
r = covarianza de ruido de medición
x = valor de interés
p = covariación del error de estimación
k = ganancia de Kalman

q = 0,125
r = 32
p = 1023 // "lo suficientemente grande como para reducirlo"
Kalman filtroX(0.125,32,1023,0);

 */
 
Kalman filtroX(0.125,10,1024,0);
Kalman filtroY(0.125,10,1024,0);
Kalman filtroZ(0.125,10,1024,9.8);
double ax_fil, ay_fil, az_fil, a_modulo;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

int16_t ax, ay, az;
int16_t gx, gy, gz;
float acceleracion_actual_x,  acceleracion_actual_y,  gravedad ;
float acceleracion_anterior_x,  acceleracion_anterior_y, radio;
float accel_ang_x, accel_ang_y;
float ang_x, ang_y, ang_x_anterior, ang_y_anterior;
float vel_x=0,vel_y=0,pos_x=0,pos_y=0;
float vel_x_anterior=0,vel_y_anterior=0,pos_x_anterior=0,pos_y_anterior=0;
double t_actual, t_anterior, dt;


// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container

VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorFloat vel;
VectorFloat pos;
float euler[3];         // [psi, theta, phi]    Euler angle container
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Wire.begin();
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    
    //Your offsets:  -1408 2160  966 68  9 25
    mpu.setXGyroOffset(68); 
    mpu.setYGyroOffset(9);
    mpu.setZGyroOffset(25);
    mpu.setXAccelOffset(-1408); 
    mpu.setYAccelOffset(2160); 
    mpu.setZAccelOffset(966); 
  
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        
        ////Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        ////Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        ///Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    // Configuracion de modo de pines como entrada y salidas. 
    pinMode(P_ECHO_D, INPUT);             
    pinMode(P_TRIGGER_D, OUTPUT);   
    pinMode(P_ECHO_A, INPUT);            
    pinMode(P_TRIGGER_A, OUTPUT);  
           
    pinMode(MOTOR_DIR, OUTPUT);           
    pinMode(MOTOR_DIR_R, OUTPUT);  
          
    pinMode(LED_MARCHA_ATRAS, OUTPUT);    
    pinMode(LED_MARCHA_ADELANTE, OUTPUT); 
    
    pinMode(MOTOR_TRACCION, OUTPUT);      
    pinMode(MOTOR_TRACCION_R, OUTPUT);  
      
    pinMode(LED_PIN, OUTPUT);
}


void loop() {
    // lee el bluetooth y almacena en estado
    if (Serial.available() > 0) {    
          estado = (char)Serial.read();
        Serial.flush();
        Serial.println(estado);
    }  

    
    if(estado == 'f')
      encendido = true;
      
    // Si esta prendido muestro la info y hago la detecion de obstaculos y veo las peticiones de movimiento.
    if( encendido ) { 
     // mostrar();
     // deteccion_de_obstaculos();
      movimiento();       
    } 
    
    // Configuraciones del acelerometro, Funca no tocar >:( 
    delay(50); 
    mpu.resetFIFO();
    if (!dmpReady) return;
    
    while (!mpuInterrupt && fifoCount < packetSize) {
      
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
       while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q); // en la posicion 0 del euler esta el angulo en radianes de eje x^y (osea visto desde arriba, seria el Z) 
    // DE ACA PARA ABAJO TOCAR: 
    // aca deberia estar el calculo correcto para obtener la posi
    delay(100);
    t_anterior = t_actual;
    t_actual = (float)micros()/1000000;
    dt = t_actual-t_anterior; //delta tiempo (x,y,z) 
    mpu.getAcceleration(&ax, &ay, &az);  // f(x,y,z)
    mpu.dmpGetGravity(&gravity, &q);
    //Filtro para el ruido
    if(ax>(-500) && ax<(500))
      ax=0;
    if(ay>(-200) && ay<(200))
      ay=0;
    if(az>(-200) && az<(200))
      az=0;
    acceleracion_anterior_y = ay_fil;
    acceleracion_anterior_x = ax_fil;
    ax_fil = (double) ax* (9.81/16384.0);
    ay_fil = (double) ay* (9.81/16384.0);
    az_fil = (double) az* (9.81/16384.0);
    ax_fil = filtroX.getFilteredValue(ax_fil);
    ay_fil = filtroY.getFilteredValue(ay_fil);
    az_fil = filtroZ.getFilteredValue(az_fil); // f(x,y,z) filtrado por kalman 
   
   // Serial.print(ax_fil);Serial.print(" ");
   // Serial.print(az_fil);Serial.println();
   // Serial.print(euler[0]);Serial.println();
    // ∫∫ f(x,y,z) filtrado dx dy dz 
    vel_y += 0.5*dt*(ay_fil+acceleracion_anterior_y);
    pos_y += 0.5*dt*(vel_y+vel_y_anterior);
    vel_x += 0.5*dt*(ax_fil+acceleracion_anterior_x);
    pos_x += 0.5*dt*(vel_x+vel_x_anterior);

/*
 * codigo rancio asi no era dani
    ax_fil = ax_fil + sin( euler[1]) * az_fil ;

    ay_fil = ay_fil + sin( euler[2])  * az_fil;

    if (ax_fil = 0xFFFFFFFF)
      ax_fil = 0;

    if (ay_fil = 0xFFFFFFFF)
      ax_fil = 0;


          
    a_modulo = sqrt ( ax_fil*ax_fil + ay_fil*ay_fil + az_fil*az_fil );
    ax_fil = ax_fil + sin( euler[1]) * a_modulo ;

    ay_fil = ay_fil + sin( euler[2]) * a_modulo;

    */

    
    a_modulo = sqrt ( ax*ax + ay*ay + az*az );
    ax_fil = ax + sin( euler[1]) * a_modulo ;

    ay_fil = ay_fil + sin( euler[2]) * a_modulo;


    Serial.print("raw_x: ");Serial.print(ax);
    Serial.print(" raw_y: ");Serial.print(ay);
    Serial.print(" raw_z: ");Serial.print(az);
    Serial.print(" modulo: ");Serial.print(a_modulo);

    /*
    Serial.print(" Acc_x: ");Serial.print(ax_fil);
    Serial.print(" Vel_x: ");Serial.print(vel_x);
    Serial.print(" Pos_x: ");Serial.print(pos_x);
    
    Serial.print(" Acc_y: ");Serial.print(ay_fil);
    Serial.print(" Vel_y: ");Serial.print(vel_y);
    Serial.print(" Pos_y: ");Serial.print(pos_y);
    */
    Serial.print(" Angulo: °");Serial.print(euler[0]);
    Serial.println();
   }
}


void movimiento(){
   switch(estado){
      case APAGAR:  //Boton apagar
        analogWrite(MOTOR_TRACCION_R, 0);
        analogWrite(MOTOR_TRACCION, 0);
        analogWrite(MOTOR_DIR, 0);
        analogWrite(MOTOR_DIR_R, 0);
        analogWrite(LED_MARCHA_ADELANTE, LOW);
        analogWrite(LED_MARCHA_ATRAS, LOW);
        encendido = false;
        break;  
      
      case ADELANTE: // Boton desplazar hacia adelante
        if( !obstaculo_D ) {
          analogWrite(MOTOR_TRACCION, velocidad);
          analogWrite(MOTOR_TRACCION_R, 0);
          digitalWrite(LED_MARCHA_ADELANTE, HIGH);
          digitalWrite(LED_MARCHA_ATRAS, LOW);
          hacia_delante = true;
          reversa = false;
        }  
      break;

      case REVERSA: // Boton desplazar hacia atras
        if( !obstaculo_T ) {
          analogWrite(MOTOR_TRACCION, 0);
          analogWrite(MOTOR_TRACCION_R, velocidad);
          digitalWrite(LED_MARCHA_ATRAS, HIGH);
          digitalWrite(LED_MARCHA_ADELANTE, LOW);
          reversa = true;
          hacia_delante = false;
        }  
        break;
        
      case PARAR:     // Boton Parar
        analogWrite(MOTOR_TRACCION, 0);
        analogWrite(MOTOR_TRACCION_R, 0);
        digitalWrite(LED_MARCHA_ADELANTE, LOW);
        digitalWrite(LED_MARCHA_ATRAS, LOW);
        hacia_delante = false;
        reversa = false;
        break;
        
      case DOBLAR_DERECHA: // Boton derecha
        analogWrite(MOTOR_DIR_R, 0);
        analogWrite(MOTOR_DIR, VEL_MAX);
        analogWrite(MOTOR_DIR_R, 0);
        analogWrite(MOTOR_DIR, 0);        
        break;  
        
      case DOBLAR_IZQUIERDA: // Boton izquierda
        analogWrite(MOTOR_DIR_R, VEL_MAX);
        analogWrite(MOTOR_DIR, 0);
        analogWrite(MOTOR_DIR_R, 0);
        analogWrite(MOTOR_DIR, 0);
        break;
       case LENTO:
          velocidad = VEL_LENTO;
          cambiarVelocidad();
          break;
      case NORMAL:
          velocidad = VEL_NORMAL;
          cambiarVelocidad();
          break;
      case RAPIDO:
          velocidad = VEL_RAPIDO;
          cambiarVelocidad();
          break;
      case MAX_VELOCIDAD:
          velocidad = VEL_MAX;
          cambiarVelocidad();
          break;
      }
}

void cambiarVelocidad(){
    if(hacia_delante){
      analogWrite(MOTOR_TRACCION, velocidad);
      analogWrite(MOTOR_TRACCION_R, 0);
    }else if (reversa){
      analogWrite(MOTOR_TRACCION, 0);
      analogWrite(MOTOR_TRACCION_R, velocidad);
     }
 }

 void deteccion_de_obstaculos(){
    
    digitalWrite(P_TRIGGER_D, LOW);
    delayMicroseconds(4);
    digitalWrite(P_TRIGGER_D, HIGH);                // genera el pulso de trigger delantero por 10us
    delayMicroseconds(10);
    duracion = pulseIn(P_ECHO_D, HIGH);             // Lee el tiempo del Echo
    distanciaD =  (duracion/29) / 2;       // calcula la distancia en centimetros

    digitalWrite(P_TRIGGER_A, LOW);
    delayMicroseconds(4);
    digitalWrite(P_TRIGGER_A, HIGH);                // genera el pulso de trigger de atras por 10us
    delayMicroseconds(10);
    duracion = pulseIn(P_ECHO_A, HIGH);             // Lee el tiempo del Echo
    distanciaT = (duracion/29) / 2;       // calcula la distancia en centimetros

    if ( distanciaD <= DISTANCIA_FRENO && distanciaD >=2 ) { // si la distancia es menor a la Distacia de freno (15)
      digitalWrite(13, HIGH);                                // Enciende LED
      analogWrite(MOTOR_TRACCION_R, 0);
      analogWrite(MOTOR_TRACCION, 0);
      obstaculo_D = true;
      Serial.println("Se ha detectado un obstaculo adelante.");
    }else 
      if( reversa && distanciaT <= DISTANCIA_FRENO && distanciaD >=2) {        // Si no hay obstaculotT
        digitalWrite(13, HIGH);                              // Enciende LED
        analogWrite(MOTOR_TRACCION_R, 0);
        analogWrite(MOTOR_TRACCION, 0);
        obstaculo_T = true;
    }else{
      obstaculo_D = false;
      obstaculo_T = false;
    }
}

void mostrar(){
      Serial.print("( ");
      Serial.print(euler[0]);
      Serial.print(" , ");
      Serial.print(aaWorld.x);
      Serial.print(" , "); 
      Serial.print(aaWorld.y);
      Serial.print(" , "); 
      Serial.print(aaWorld.z);
      Serial.println(" )");    
}
/*
void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}
void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
}
*/
