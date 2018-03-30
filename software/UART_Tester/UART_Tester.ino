
int32_t outByte = 99999;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  
  Serial.println("Setup finished!");
}

// the loop function runs over and over again forever
void loop() {

    Serial1.write(outByte);
}

void serialEvent1(){
    int32_t inByte = Serial1.read();
    Serial.print("Mega received: ");
    Serial.println(inByte);

    //Serial1.write(outByte);
    outByte--;
}

