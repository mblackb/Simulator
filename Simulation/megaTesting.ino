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
    Demo Values for Simulator
    Throttle : float (-1 to 1)
    Steering : float (-1 to 1)
    Brakes: float (0 for off 0.3 for on)
*/

void driveFrontandBack(){


    //start driving, then turn right after 5 seconds, continue driving
    ToggleLight();
    updateThrottle(1); 
    ToggleLight();  

    delay(5000);

    ToggleLight();
    updateThrottle(-1); 
    ToggleLight();   

    delay(5000);
}

void RandomDemo(){

    //start driving, then turn right after 5 seconds, continue driving
    ToggleLight();
    updateThrottle(.2); 
    delay(5000);
    updateSteering(.5); 
    ToggleLight();    
    
    //Wait 5 seconds and then stop throttle and apply brakes
    ToggleLight();
    digitalWrite(13, HIGH);  
    updateThrottle(0);
    updateBrake(0.3); 
    ToggleLight();  

    //Wait 5 seconds and start again
    delay(5000);    
    updateBrake(0);
}

void ToggleLight(){
    if (!lightOn){
        digitalWrite(13, HIGH); 
    } else {
        digitalWrite(13, LOW); 
    }

    lightOn = !lightOn;
}

void updateThrottle(float value){
    Serial.println(0); //mapped to throttle command in simulator
    Serial.println(value);
}

void updateSteering(float value){
    Serial.println(1); //mapped to steering command in simulator
    Serial.println(value);
}

void updateBrake(float value){
    Serial.println(2); //mapped to braking command in simulator
    Serial.println(value);
}