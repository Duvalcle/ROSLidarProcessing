String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int index = 0; //index the readen data; [0-7] correspond to switch case
//Union data to link representation of float
union floatX_bytes {
     float val;
     unsigned char bytes[sizeof(float)];
} dataX,dataY,dataRay;
//Union data to link representation of int
union int_bytes {
     int val;
     unsigned char bytes[sizeof(int)];
} dataI;

void setup() {
  // initialize serial:
  Serial.begin(115200);
  initData();
}

void loop() {
  if (stringComplete) {
    //Ressend data over Serial to check decoding
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

/*
  Init Union data structure for another set of data
*/
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
    //Switch between cases of byte reception (if it is the first or the last one...
    //Respect same spec as in ../../src/hokuyo_processing.cpp
    switch (index){
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
    index++;
    //end of data reception
    if (inChar == '\n') {
      stringComplete = true;
      index = 0;
    } 
  }
}


