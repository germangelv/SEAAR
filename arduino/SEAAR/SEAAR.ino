#include "Wire.h"
#include <Servo.h>
#include <Ultrasonic.h>                


// PINES:
//0 y 1 para tx y rx del bluetooth
// 2 pin de interrupcion para el mpu ??
//LIBRES:   6  
//

#define P_ECHO_D            3     //Pin 03 como (ECHO PWM ) para el Ultrasonido Delantero
#define P_TRIGGER_D         4     //Pin 04 como (TRIGGER ) para el Ultrasonido Delantero
#define SERVO               5     //Pin 05 como SERVO
#define LED_MARCHA_ADELANTE 7     //Pin 07 LED de adelante
#define LED_MARCHA_ATRAS    8     //Pin 08 LED de atras
#define MOTOR_TRACCION      9     //Pin 09 Motor hacia adelante
#define MOTOR_TRACCION_R    10    //Pin 10 Motor hacia atras
#define P_ECHO_A            11    //Pin 11 como (P_ECHO PWM ) para el Ultrasonido Trasero
#define P_TRIGGER_A         12    //Pin 12 como (P_TRIGGER_) para el Ultrasonido Trasero
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
#define ENDEREZAR 'x'
#define APAGAR 'o'
#define ENCENDER 'f'
#define MAX_VELOCIDAD 'l'
#define OSCURO ''
#define CLARO ''
//Velocidades 
#define VEL_NORMAL 100
#define VEL_MAX 150   
Servo servoMotor;      
Ultrasonic ultrasonicT(P_TRIGGER_A,P_ECHO_A,30000); 
Ultrasonic ultrasonicD(P_TRIGGER_D,P_ECHO_D,30000); 


 
unsigned int velocidad = VEL_NORMAL;    // Velocidad de los motores (MACROS)
char estado = APAGAR;                   // Inicia detenido
long duracion, distanciaD, distanciaT;  // Para Calcular distacia
//Booleanos de estados
boolean obstaculo_D;                    // Determina si hay obstaculo DELANTERO
boolean obstaculo_T;                    // Determina si hay obstaculo TRASERO
boolean encendido=false; 
boolean hacia_delante=false;
boolean reversa=false;



void setup() {
  
    Wire.begin();
    Serial.begin(9600);
    //Servo
    servoMotor.attach(SERVO, 650, 2400);
    // MPU9250
    
    // Configuracion de modo de pines como entrada y salidas. 
    pinMode(P_ECHO_D, INPUT);             
    pinMode(P_TRIGGER_D, OUTPUT);   
    pinMode(P_ECHO_A, INPUT);            
    pinMode(P_TRIGGER_A, OUTPUT);  
    
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
    }  
    if(estado == 'f')
      encendido = true;
      
    // Si esta prendido muestro la info y hago la detecion de obstaculos y veo las peticiones de movimiento.
    if( encendido ) { 
     // mostrar();
      deteccion_de_obstaculos();
      movimiento();       
    } 
}


void movimiento(){
   switch(estado){
      case APAGAR:  //Boton apagar
        analogWrite(MOTOR_TRACCION_R, 0);
        analogWrite(MOTOR_TRACCION, 0);
        servoMotor.write(0);
        analogWrite(LED_MARCHA_ADELANTE, LOW);
        analogWrite(LED_MARCHA_ATRAS, LOW);
        encendido = false;
        break;  
      
      case ADELANTE: // Boton desplazar hacia adelante
        if( !obstaculo_D ) {
          velocidad = VEL_NORMAL;
          analogWrite(MOTOR_TRACCION, velocidad);
          analogWrite(MOTOR_TRACCION_R, 0);
          digitalWrite(LED_MARCHA_ATRAS, LOW);
          hacia_delante = true;
          reversa = false;
        }  
      break;
      
      case MAX_VELOCIDAD:
        if( !obstaculo_D ) {
          velocidad = VEL_MAX;
          analogWrite(MOTOR_TRACCION, velocidad);
          analogWrite(MOTOR_TRACCION_R, 0);
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
        servoMotor.write(0); //angulo
        break;  
        
      case ENDEREZAR: // Boton dirrecion 
        servoMotor.write(0); //angulo
        break;
        
      case DOBLAR_IZQUIERDA: // Boton izquierda
        servoMotor.write(0); //angulo
        break;
        
      case OSCURO: // Sensor de luz
        digitalWrite(LED_MARCHA_ADELANTE, HIGH);
        break;  
      case CLARO: // Sensor de luz 
        digitalWrite(LED_MARCHA_ADELANTE, LOW);
        break;  

}

 void deteccion_de_obstaculos(){
     
    if ( hacia_delante && ultrasonicD.Ranging(CM) <= DISTANCIA_FRENO) { // si la distancia es menor a la Distacia de freno (15)
      digitalWrite(13, HIGH);                                // Enciende LED
      analogWrite(MOTOR_TRACCION_R, 0);
      analogWrite(MOTOR_TRACCION, 0);
      obstaculo_D = true;
      Serial.println("Se ha detectado un obstaculo adelante.");
    }else 
      if( reversa && ultrasonicT.Ranging(CM) <= DISTANCIA_FRENO ) {        // Si no hay obstaculotT
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
  // coordenadas separadas por espacio ej: 'x y'
  //Serial.print(x);
  //Serial.print(" ");
  //Serial.println(y);
  
  }
