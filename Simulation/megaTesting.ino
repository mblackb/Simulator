//For testing communication output from python interface


void setup() {

    pinMode(13, OUTPUT);

    Serial.begin(115200);
    //while (Serial.available() <= 0) {}; //wait for it to send a message before entering loop
    
}

void loop(){
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);                       // wait for a second
}