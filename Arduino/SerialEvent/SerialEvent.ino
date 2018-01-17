/*
  Serial Event example
 
 When new serial data arrives, this sketch adds it to a String.
 When a newline is received, the loop prints the string and 
 clears it.
 
 A good test for this is to try it with a GPS receiver 
 that sends out NMEA 0183 sentences. 
 
 Created 9 May 2011
 by Tom Igoe
 
 This example code is in the public domain.
 
 http://www.arduino.cc/en/Tutorial/SerialEvent
 
 */
 
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int flag = 0;
union floatX_bytes {
     float val;
     unsigned char bytes[sizeof(float)];
} dataX,dataY,dataRay;
union int_bytes {
     int val;
     unsigned char bytes[sizeof(int)];
} dataI;

void setup() {
  // initialize serial:
  Serial.begin(115200);
  initData();
  // reserve 200 bytes for the inputString:
  //inputString.reserve(200);
}

void loop() {
  // print the string when a newline arrives:
 /* while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
   Serial.print(inChar);
  }
  */
  if (stringComplete) {
    //Serial.println(inputString);
    Serial.print("Id : ");
    Serial.print(dataI.val);
    Serial.print("|x : ");
    Serial.print(dataX.val,3);
    Serial.print("|y : ");
    Serial.print(dataY.val,3);
    Serial.print("|r : ");
    Serial.println(dataRay.val,3);    
    // clear the string:
    
    inputString = "";
    initData;
    stringComplete = false;
  }
}

void initData(){
  dataX.val = 0;
  dataY.val = 0;
  dataRay.val = 0;
  dataI.val = 0;
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  
  while (Serial.available()) {
    // get the new byte:
    unsigned char inChar = (unsigned char)Serial.read(); 
    //Serial.println(inChar);
    // add it to the inputString:
    switch (flag){
    case 0:
      dataI.bytes[0]=inChar;
      break;
    case 1:
      dataX.bytes[3]=inChar;
      break;
    case 2:
      dataX.bytes[2]=inChar;
      break;
    case 3:
      dataY.bytes[3]=inChar;
      break;
    case 4:
      dataY.bytes[2]=inChar;
      break;
    case 5:
      dataRay.bytes[3]=inChar;
      break;
    case 6:
      dataRay.bytes[2]=inChar;
      break;
    }
    flag++;
    //inputString += inChar;
    
    //Serial.println("received");
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      flag = 0;
    } 
  }
}


