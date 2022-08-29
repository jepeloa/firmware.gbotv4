int steering_angle_value=0;  //variable que almacena el angulo de la direccion
void steering_angle (){        //funcion que calcula la conversion del angulo
  int dir_pos_0;
  int dir_pos_1;
  int dir_pos_2;
  int dir_pos_3;
  int prom_dir=0;
// Serial.println(steering_angle_value);
//-----------------------Rueda delantera 1 (ver izquierda o derecha)
prom_dir=0;
for (int i = 0; i < 10; i++) {   //Promedio de los valores de direccion medidos, para sacar el ruido
  dir_pos_0=analogRead(POT_SENSOR_0);
  prom_dir=prom_dir+dir_pos_0;
  }
  prom_dir=prom_dir/10;
 if (steering_angle_value> MAX_ENCODER_VALUE) {  //si el valor es mayor a este argumento estoy con el servo totalmente retraido
      analogWrite(STEERING_PIN_PWM_BACK_RIGHT,0);
      return;
      }  
      if (steering_angle_value< MIN_ENCODER_VALUE) {  //si el valor es mayor a este argumento estoy con el servo totalmente retraido
      analogWrite(STEERING_PIN_PWM_BACK_RIGHT,0);
      return;
      }
 if (steering_angle_value < prom_dir) {   //si el valor del argumento es menor al de la posicion el servo debe retraerse
      digitalWrite(DIR_0,LOW);
      delay(1);
      analogWrite(STEERING_PIN_PWM_BACK_RIGHT,STEERING_SPEED);  
      }

 else if(steering_angle_value > prom_dir+10) {             //agrego un gap para que no se solape permanentemente lo mismo que el caso anterior pero invirtiendo los estados logicos, el servo debe extenderse
      digitalWrite(DIR_0,HIGH);
      delay(1);
      analogWrite(STEERING_PIN_PWM_BACK_RIGHT,STEERING_SPEED);  
      }
 else {
       analogWrite(STEERING_PIN_PWM_BACK_RIGHT,0); 
 }

//-----------------------------------------------------------------------
//---------Rueda delantera 2 (ver izquierda o derecha)
  //dir_pos_1=analogRead(POT_SENSOR_1)+91; //le resto 90 para que queden alineadas las ruedas (atras)
  prom_dir=0;
  for (int i = 0; i < 10; i++) {   //Promedio de los valores de direccion medidos, para sacar el ruido
  dir_pos_1=analogRead(POT_SENSOR_1)+93;
  prom_dir=prom_dir+dir_pos_1;
  }
  prom_dir=prom_dir/10;
 if (steering_angle_value> MAX_ENCODER_VALUE) {  //si el valor es mayor a este argumento estoy con el servo totalmente retraido
      analogWrite(STEERING_PIN_PWM_BACK_LEFT,0);
      return;
      }  
      if (steering_angle_value< MIN_ENCODER_VALUE) {  //si el valor es mayor a este argumento estoy con el servo totalmente retraido
      analogWrite(STEERING_PIN_PWM_BACK_LEFT,0);
      return;
      }
 if (steering_angle_value < prom_dir) {   //si el valor del argumento es menor al de la posicion el servo debe retraerse
      digitalWrite(DIR_1,LOW);
      delay(1);
      analogWrite(STEERING_PIN_PWM_BACK_LEFT,STEERING_SPEED);  
      }

 else if(steering_angle_value > prom_dir+10) {                     //agrego un gap para que no se solape permanentemente lo mismo que el caso anterior pero invirtiendo los estados logicos, el servo debe extenderse
      digitalWrite(DIR_1,HIGH);
      delay(1);
      analogWrite(STEERING_PIN_PWM_BACK_LEFT,STEERING_SPEED);  
      }
 else {
       analogWrite(STEERING_PIN_PWM_BACK_LEFT,0); 
 }

//-----------------------------------------------------------------------

       
}
void update_steering() {
 if (!moving){
    analogWrite(STEERING_PIN_PWM_BACK_LEFT,0);
    analogWrite(STEERING_PIN_PWM_BACK_RIGHT,0);
    return;
  }

else{

  steering_angle();
}
  }
