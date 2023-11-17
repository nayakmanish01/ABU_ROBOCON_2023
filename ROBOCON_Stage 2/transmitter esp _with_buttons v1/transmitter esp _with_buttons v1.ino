#include <Ps3Controller.h>
#include <ESP32_Servo.h>


Servo myservo_yaw_feeding; 
Servo myservo1_yaw_feeding;
Servo myservo_feeding; 
Servo myservo1_feeding;
int up=0;
int down=0;

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t x;
  int16_t y;
};
SEND_DATA_STRUCTURE data;

#define actuator_dir 5
#define actuator_pwm 22

void notify()
{
    //--- Digital cross/square/triangle/circle button events ---
    if( Ps3.event.button_down.cross ){
      Serial.println("the cross button");
    
      myservo_yaw_feeding.write(0);
      myservo1_yaw_feeding.write(173);
      delay(1000);
      myservo_yaw_feeding.write(173);
      myservo1_yaw_feeding.write(0);
      delay(1000);
    }
        
    if( Ps3.event.button_up.cross )
        Serial.println("Released the cross button");
    if( Ps3.event.button_up.square )
      

      

    if( Ps3.event.button_down.triangle )
        { 
      myservo_feeding.write(0);
      myservo1_feeding.write(173);
      delay(1000);
      myservo_feeding.write(173);
      myservo1_feeding.write(0);
      delay(1000);
         
          Serial.println("Started pressing the triangle button");
        }
       
            
         
        
    if( Ps3.event.button_up.triangle )
        Serial.println("Released the triangle button");

    if( Ps3.event.button_down.circle ){
      
        Serial.println("Started pressing the circle button");
    }
    if( Ps3.event.button_up.circle )
        Serial.println("Released the circle button");

    //--------------- Digital D-pad button events --------------
    if( Ps3.event.button_down.up )
        Serial.println("Started pressing the up button");
    if( Ps3.event.button_up.up )
        Serial.println("Released the up button");

    if( Ps3.event.button_down.right )
        Serial.println("Started pressing the right button");
    if( Ps3.event.button_up.right )
        Serial.println("Released the right button");

    if( Ps3.event.button_down.down )
        Serial.println("Started pressing the down button");
    if( Ps3.event.button_up.down )
        Serial.println("Released the down button");

    if( Ps3.event.button_down.left )
        Serial.println("Started pressing the left button");
    if( Ps3.event.button_up.left )
        Serial.println("Released the left button");

    //------------- Digital shoulder button events -------------
    if( Ps3.event.button_down.l1 )
        Serial.println("Started pressing the left shoulder button");
    if( Ps3.event.button_up.l1 )
        Serial.println("Released the left shoulder button");

    if( Ps3.event.button_down.r1 )
        Serial.println("Started pressing the right shoulder button");
    if( Ps3.event.button_up.r1 )
        Serial.println("Released the right shoulder button");

    //-------------- Digital trigger button events -------------
    if( Ps3.event.button_down.l2 )
        Serial.println("Started pressing the left trigger button");
    if( Ps3.event.button_up.l2 )
        Serial.println("Released the left trigger button");

    if( Ps3.event.button_down.r2 )
        Serial.println("Started pressing the right trigger button");
    if( Ps3.event.button_up.r2 )
        Serial.println("Released the right trigger button");

    //--------------- Digital stick button events --------------
    if( Ps3.event.button_down.l3 )
        Serial.println("Started pressing the left stick button");
    if( Ps3.event.button_up.l3 )
        Serial.println("Released the left stick button");

    if( Ps3.event.button_down.r3 )
        Serial.println("Started pressing the right stick button");
    if( Ps3.event.button_up.r3 )
        Serial.println("Released the right stick button");

    //---------- Digital select/start/ps button events ---------
    if( Ps3.event.button_down.select )
        Serial.println("Started pressing the select button");
    if( Ps3.event.button_up.select )
        Serial.println("Released the select button");

    if( Ps3.event.button_down.start )
        Serial.println("Started pressing the start button");
    if( Ps3.event.button_up.start )
        Serial.println("Released the start button");

    if( Ps3.event.button_down.ps )
        Serial.println("Started pressing the Playstation button");
    if( Ps3.event.button_up.ps )
        Serial.println("Released the Playstation button");


    //---------------- Analog stick value events ---------------
   if( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 2 ){
      //  Serial.print("Moved the left stick:");
      //  Serial.print(" x="); 
      //  Serial.print(Ps3.data.analog.stick.lx, DEC);
      //  Serial.print(" y="); 
      //  Serial.print(Ps3.data.analog.stick.ly, DEC);
      //  Serial.println();
       int X= Ps3.data.analog.stick.lx;
       int Y = -Ps3.data.analog.stick.ly;
       data.x=map(X,-127,127,0,255);
       data.y=map(Y,-127,127,0,255);
      Serial2.write((uint8_t*)&data, sizeof(data)); // Send the PS3Data struct as a byte array
     delay(100);
Serial.print(" x="); Serial.println(X);
Serial.print(" Y="); Serial.println(Y);

    }

   if( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2 ){
       Serial.print("Moved the right stick:");
       Serial.print(" x="); Serial.print(Ps3.data.analog.stick.rx, DEC);
       Serial.print(" y="); Serial.print(Ps3.data.analog.stick.ry, DEC);
       Serial.println();
   }

   //--------------- Analog D-pad button events ----------------
   if( abs(Ps3.event.analog_changed.button.up) ){
       Serial.print("Pressing the up button: ");
       Serial.println(Ps3.data.analog.button.up, DEC);
       up=Ps3.data.analog.button.up;
       Serial.print(up);
       if(up>0 && up<=255){
           digitalWrite(actuator_dir, HIGH);
          digitalWrite(actuator_pwm, HIGH);
       }
       else{
           digitalWrite(actuator_dir, LOW);
          digitalWrite(actuator_pwm, LOW);
       }
   }
 

   if( abs(Ps3.event.analog_changed.button.right) ){
       Serial.print("Pressing the right button: ");
       Serial.println(Ps3.data.analog.button.right, DEC);
   }

   if( abs(Ps3.event.analog_changed.button.down) ){
       Serial.print("Pressing the down button: ");
       Serial.println(Ps3.data.analog.button.down, DEC);
       down=Ps3.data.analog.button.down;
           if(down>0 && down<=255){
           digitalWrite(actuator_dir, LOW);
          digitalWrite(actuator_pwm, HIGH);
       }
       else{
           digitalWrite(actuator_dir, LOW);
          digitalWrite(actuator_pwm, LOW);
       }
   }

   if( abs(Ps3.event.analog_changed.button.left) ){
       Serial.print("Pressing the left button: ");
       Serial.println(Ps3.data.analog.button.left, DEC);
   }

   //---------- Analog shoulder/trigger button events ----------
   if( abs(Ps3.event.analog_changed.button.l1)){
       Serial.print("Pressing the left shoulder button: ");
       Serial.println(Ps3.data.analog.button.l1, DEC);
   }

   if( abs(Ps3.event.analog_changed.button.r1) ){
       Serial.print("Pressing the right shoulder button: ");
       Serial.println(Ps3.data.analog.button.r1, DEC);
   }

   if( abs(Ps3.event.analog_changed.button.l2) ){
       Serial.print("Pressing the left trigger button: ");
       Serial.println(Ps3.data.analog.button.l2, DEC);
   }

   if( abs(Ps3.event.analog_changed.button.r2) ){
       Serial.print("Pressing the right trigger button: ");
       Serial.println(Ps3.data.analog.button.r2, DEC);
   }

   //---- Analog cross/square/triangle/circle button events ----
   if( abs(Ps3.event.analog_changed.button.triangle)){
       Serial.print("Pressing the triangle button: ");
       Serial.println(Ps3.data.analog.button.triangle, DEC);
  
   }

   if( abs(Ps3.event.analog_changed.button.circle) ){
       Serial.print("Pressing the circle button: ");
       Serial.println(Ps3.data.analog.button.circle, DEC);
   }

   if( abs(Ps3.event.analog_changed.button.cross) ){
       Serial.print("Pressing the cross button: ");
       Serial.println(Ps3.data.analog.button.cross, DEC);
   }

   if( abs(Ps3.event.analog_changed.button.square) ){
       Serial.print("Pressing the square button: ");
       Serial.println(Ps3.data.analog.button.square, DEC);
   }

    
}

void onConnect(){
    Serial.println("Connected.");
}

void setup()
{
    Serial.begin(9600);
    pinMode(actuator_dir,OUTPUT);
    pinMode(actuator_pwm,OUTPUT);

    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin("00:1a:7d:da:71:10");
    myservo_yaw_feeding.attach(14,500,2500);
    myservo1_yaw_feeding.attach(27,500,2500);
    myservo_feeding.attach(12,500,2500);
    myservo1_feeding.attach(13,500,2500);
    Serial2.begin(9600, SERIAL_8N1, 16, 17);
    Serial.println("Ready.");
}

void loop()
{
    if(!Ps3.isConnected())
        return;
    
  
//    Serial.print("x ");
//  Serial.println(data.x);
//  Serial.print("y ");
// Serial.println(data.y);

    //------ Digital cross/square/triangle/circle buttons ------
    if( Ps3.data.button.cross && Ps3.data.button.down )
        Serial.println("Pressing both the down and cross buttons");
    if( Ps3.data.button.square && Ps3.data.button.left )
        Serial.println("Pressing both the square and left buttons");
    if( Ps3.data.button.triangle && Ps3.data.button.up )
        Serial.println("Pressing both the triangle and up buttons");
    if( Ps3.data.button.circle && Ps3.data.button.right )
        Serial.println("Pressing both the circle and right buttons");

    if( Ps3.data.button.l1 && Ps3.data.button.r1 )
        Serial.println("Pressing both the left and right bumper buttons");
    if( Ps3.data.button.l2 && Ps3.data.button.r2 )
        Serial.println("Pressing both the left and right trigger buttons");
    if( Ps3.data.button.l3 && Ps3.data.button.r3 )
        Serial.println("Pressing both the left and right stick buttons");
    if( Ps3.data.button.select && Ps3.data.button.start )
        Serial.println("Pressing both the select and start buttons");
     
        delay(2000);
       
}