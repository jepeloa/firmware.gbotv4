/*********************************************************************
 *  ROSArduinoBridge
 
  
 *********************************************************************/

#define USE_BASE     // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
   /* The Pololu VNH5019 dual motor driver shield */
   //#define L298_MOTOR_DRIVER

   /* The Pololu MC33926 dual motor driver shield */
   //#define POLOLU_MC33926

   /* The RoboGaia encoder shield */
   #define ARDUINO_ENC_COUNTER
   
   /* Encoders directly attached to Arduino board */
   //#define ARDUINO_ENC_COUNTER

   /* Motor de una ebike para gbot*/
   #define ebike_motor
#endif

//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
//#undef USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE     9600
 //steering (only for GBOT)
 //pines de potenciometros de las ruedas (analogicos)
#define POT_SENSOR_0 0   //A0 Rueda derecha trasera
#define POT_SENSOR_1 1   //A1
#define POT_SENSOR_2 2   //A2
#define POT_SENSOR_3 3   //A3
//pines de direccion de las ruedas
#define DIR_0 7   //wheel_0 (rueda trasera derecha)
#define DIR_1 8   //wheel_1
//--------------------------------------------------MARCHA ATRAS
#define PHASE_1 9   //fase sentido adelante
#define PHASE_2 12   //fase sentido marcha atras
//-------------------------------------------------MARCHA ATRAS
#define STEERING_PIN_PWM_BACK_RIGHT 11  //rueda trasera
#define STEERING_PIN_PWM_BACK_LEFT 10  //rueda trasera
#define STEERING_SPEED 70 //valor de velocidad de giro de las ruedas
#define MAX_ENCODER_VALUE 800
#define MIN_ENCODER_VALUE 450     //valores maximos y minimos de las lecturas del enconder de direccion
#define CAL_STERING_CONST 50       //valor de la constante de caibracion de la direccion
#define DISTANCE_WHEELS 1 //distancia en metros entre ruedas solo para el caso de GBOT
/* Maximum PWM signal */
#define MAX_PWM        2000
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"
#endif

#ifdef ebike_motor    //Aca se define el puerto I2C por software para que se pueda trabajar con los aceleradores DAC
 
#define I2C_TIMEOUT 1000
#define I2C_PULLUP 1
#define SDA_PORT PORTD
#define SDA_PIN 5 // = pin 5 digital
#define SCL_PORT PORTD
#define SCL_PIN 6 // = pin 6 digital
#include <SoftI2CMaster.h>
#define I2C_7BITADDR_0 0x62 // acelerador derecha
#define I2C_7BITADDR_1 0x63 // acelerador izquierda conectado al mismo puerto I2C
#define ADDRLEN 1 // address length, usually 1 or 2 bytes
#define MCP4726_CMD_WRITEDAC            (0x40)  // Writes data to the DAC
#define MCP4726_CMD_WRITEDACEEPROM      (0x60)  // Writes data to the DAC and the EEPROM (persisting the assigned value after reset)
#endif

#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
  #include "diff_controller.h"

  #include "steering.h"   //direccion ackermann GBOT

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;
  
  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 2000

 
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif


/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;
unsigned char reverse_left=0; //Variable que almacena el valor de la direccion del movimiento rueda izquierda es muy importante para la rueda de la ebike
unsigned char reverse_rigth=0;//Variable que almacena el valor de la direccion del movimiento rueda derecha es muy importante para la rueda de la ebike


// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;
bool dir_=1; 
bool marcha=0;
/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

void forward_backward(bool dir){
       if (dir==1){
      
      //Serial.println("Opening phases of the motors");
      digitalWrite(PHASE_1, LOW);
      digitalWrite(PHASE_2, HIGH);
      }
      else{
      digitalWrite(PHASE_1, HIGH);
      digitalWrite(PHASE_2, LOW);
      }
}



/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  int dir_pos;
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIRECTION_CHANGE:    //conmando para mover la direccion de forma manual
      Serial.println("OK");  //tengo que agregar los comandos aca
      break;
  case DIGITAL_WRITE:    //esto lo voy a usar para cambiar el sentido de marcha. 
    if (arg1==0){
      marcha=0;       
    }
    if (arg1==1){
      marcha=1;        
    }
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING:
    Serial.println(Ping(arg1));
    break;
#ifdef USE_SERVOS
  case SERVO_WRITE:
    //Serial.println(arg2);
    //servos[arg1].setTargetPosition(arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].getServo().read());
    break;
#endif
    
#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
   case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    //esta es la rutina para manejo de marcha y motores
    //la funcion forward_backward setea las fases, si hay un cambio de marcha hay un delay de 500ms para volver a activar los controladores
    //
    if (marcha==0){
      
    }
    else{
      arg1=-arg1;
      arg2=-arg2;
    }
    
    if ((arg1 < -1 && arg2 > 0) || (arg1 > 0 && arg2 < 1)) {  //en el caso que las velocidades sean de distinto signo solo se gira la direccion
    }
    
    
    if (arg1 < 0 && arg2 < 0) {
    if (dir_==0){
       setMotorSpeeds(0, 0);
      delay(500);
     }
     forward_backward(1);
     delay(20);
     dir_=1;
      
    }
    if (arg1 > 0 && arg2 > 0) {
    if (dir_==1){
       setMotorSpeeds(0, 0);
      delay(500);
     }
     forward_backward(0);
     delay(20);
     dir_=0;
      
    }

    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
      delay(30);
      digitalWrite(PHASE_1, LOW);   //desactivo ambas fases
      digitalWrite(PHASE_2, LOW);   //desactivo ambas fases
    }
    else moving = 1;

    
   // leftPID.TargetTicksPerFrame = int((arg1+arg2)/2);  //en el caso de ackermann la velocidad de las ruedas es la suma de las dos
   // rightPID.TargetTicksPerFrame = int((arg1+arg2)/2); //lo mismo que arriba
  
    leftPID.TargetTicksPerFrame = arg1;  //en el caso de ackermann la velocidad de las ruedas es la suma de las dos
    rightPID.TargetTicksPerFrame = arg2; //lo mismo que arriba
  
    steering_angle_value=int(CAL_STERING_CONST*(arg1-arg2)+550); //sumo el offset del centro de la direccion la ecuacion va a ser K(arg1-arg2)+370
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);
#ifdef ebike_motor
   if (!i2c_init()) Serial.println("I2C init failed");
   pinMode(PHASE_1, OUTPUT);  //establezco el pin de direccion como salida (cable gris conectado en 4
   pinMode(PHASE_2, OUTPUT);  //establezco el pin de PWM como salida digital (no voy a hacer control de velocidad) pin digital 5 cable verde
   pinMode(POT_SENSOR_0, INPUT);  //entrada del encoder de la direccion atras derecha
   pinMode(POT_SENSOR_1, INPUT);  //entrada del encoder de la direccion atras izquierda
   pinMode(POT_SENSOR_2, INPUT);
   pinMode(POT_SENSOR_3, INPUT);
   pinMode(DIR_0, OUTPUT);  //direccion atras derecha
   pinMode(DIR_1, OUTPUT);  //direccion atras izquierda
   pinMode(STEERING_PIN_PWM_BACK_RIGHT, OUTPUT);  //pwm  atras derecha
   pinMode(STEERING_PIN_PWM_BACK_LEFT, OUTPUT);  //direccion atras izquierda
   pinMode(10, OUTPUT); //pwm atras izquierda
#endif
// Initialize the motor controller if used */
#ifdef USE_BASE
  #ifdef ARDUINO_ENC_COUNTER
    //set as inputs
    DDRD &= ~(1<<LEFT_ENC_PIN_A);
    DDRD &= ~(1<<LEFT_ENC_PIN_B);https://github.com/jepeloa/firmware.gbotv4/pull/new/inversion_marcha
    DDRC &= ~(1<<RIGHT_ENC_PIN_A);
    DDRC &= ~(1<<RIGHT_ENC_PIN_B);
    
    //enable pull up resistors
    PORTD |= (1<<LEFT_ENC_PIN_A);
    PORTD |= (1<<LEFT_ENC_PIN_B);
    PORTC |= (1<<RIGHT_ENC_PIN_A);
    PORTC |= (1<<RIGHT_ENC_PIN_B);
    
    // tell pin change mask to listen to left encoder pins
    PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
    // tell pin change mask to listen to right encoder pins
    PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);
    
    // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
    PCICR |= (1 << PCIE1) | (1 << PCIE2);
  #endif
  initMotorController();
  resetPID();
#endif

/* Attach servos if used */
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) {
      servos[i].initServo(
          servoPins[i],
          stepDelay[i],
          servoInitPosition[i]);
    }
  #endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
    update_steering();    //aca llamo a la funcion que actualiza el estado de la direccion only for GBOT
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0);
    delay(30);
    digitalWrite(PHASE_1, LOW);   //desactivo ambas fases
    digitalWrite(PHASE_2, LOW);   //desactivo ambas fases
    moving = 0;
    
  }
#endif

// Sweep servos
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif
}
