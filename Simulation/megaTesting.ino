/*
For testing serial communication between carla and an arduino using my own
personal MEGA for these tests. Shows various calls the arduino can make to the
simulator
*/

boolean lightOn = false;

void setup() {
    
    pinMode(13, OUTPUT);

    Serial.begin(115200);
    ToggleLight();
    while(!Serial.available() ){}
    ToggleLight();
}

void loop(){

   RandomDemo();

}

/*
    Demo Values for Simulator all 8 bits
    Throttle : float (0 to 255) 
    Steering : float (-128 to 127)
    Brakes: float (0 to 255)
*/

void driveFrontandBack(){


    //start driving, then turn right after 5 seconds, continue driving
    ToggleLight();
    updateThrottle(127); 
    ToggleLight();  

    delay(5000);

    ToggleLight();

    ToggleLight();   

    delay(5000);
}


//Test method for driving the vehicle
void RandomDemo(){

    //start driving, then turn right after 5 seconds, continue driving
    ToggleLight();
    updateThrottle(127); //Half speed
    delay(5000);
    updateSteering(60); //Half left
    ToggleLight();    
    
    //Wait 5 seconds and then stop throttle and apply brakes
    ToggleLight(); 
    updateThrottle(0);
    ToggleLight();  

    //Wait 5 seconds and turn off the brake and set steering straight
    delay(5000);    
    updateSteering(0);
}




//Toggle the light on pin 13
void ToggleLight(){
    if (!lightOn){
        digitalWrite(13, HIGH); 
    } else {
        digitalWrite(13, LOW); 
    }

    lightOn = !lightOn;
}

void updateThrottle(int value){
    Serial.write(0); //mapped to throttle command in simulator
    Serial.write(value);
}

void updateSteering(int value){
    Serial.write(1); //mapped to steering command in simulator
    Serial.write(value);
}

void updateBrake(int value){
    Serial.write(2); //mapped to braking command in simulator
    Serial.write(value);
}