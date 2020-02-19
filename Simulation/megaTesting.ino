/*
For testing serial communication between carla and an arduino using my own
personal MEGA for these tests. Shows various calls the arduino can make to the
simulator
*/


void setup() {

    pinMode(13, OUTPUT);

    Serial.begin(115200);
    digitalWrite(13, HIGH);
    while(!Serial.available() ){}
    digitalWrite(13, LOW);
}

void loop(){

    //start driving, then turn right after 2 seconds, continue driving
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    updateThrottle(1); // Throttle : float (-1 to 1)
    delay(2000);
    updateSteering(.5); //Steering : float (-1 to 1)
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    
    //Wait 5 seconds and then stop throttle and apply brakes
    delay(5000);
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    updateThrottle(0);
    updateBrake(0.3); //Brakes: float (0 for off 0.3 for on)
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW

    //Wait 5 seconds and start again
    delay(5000);    

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