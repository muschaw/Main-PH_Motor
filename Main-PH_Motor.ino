#define SensorPin A4            //pH meter Analog output to Arduino Analog Input 2
#define PhdownlivePin 5 
#define PhdowngndPin 6
#define PhuplivePin 10
#define PhupgndPin 11
          
#define PhDownDelayOriginal 2500         // Initial delay betwen PH DOWN pump
#define PhUpDelayOriginal 5000          // Initial delay betwen PH UP pump

#define SetPHrate 60000           // Internal for PH Pump ( default 1 min = 60000)
#define samplingInterval 50       // Interval for PH Probe
#define ArrayLenth  100           // Resolution of collection
#define Vcc 5.01                  // input VCC
#define PHoffset 3.52             // calculated per probe
#define voltageoffset 0.021       // ADC probe massured offset

#define configuredPH 5.70              // Target PH
#define PHdiff 0.10                    // Target PH offset check

int pHArray[ArrayLenth];          //Store the average value of the sensor feedback
int pHArrayIndex = 0;             //Clear index 
float PHValue,voltage;            //initialize PH and volate arrays
int PHupcount = 0;                //counter for PH UP times since boot
int PHdowncount = 0;              //counter for PH DOWN times since boot

int PhDownDelay = PhDownDelayOriginal;                                                      // Initialize working variable
int PhUpDelay = PhUpDelayOriginal;               

int PhDownDelayMin = PhDownDelayOriginal / 2;                                               // Initialize working Min variable    
int PhUpDelayMin = PhUpDelayOriginal / 2;               

int PhUpMaxDelay = PhUpDelayOriginal * 2;                                                   // Initialize working Max variable   
int PhDownMaxDelay = PhDownDelayOriginal * 2;     


void setup(void){

  pinMode(PhdownlivePin, OUTPUT);                                                             // Initialize Pump pins
  pinMode(PhdowngndPin, OUTPUT);
  pinMode(PhuplivePin, OUTPUT);
  pinMode(PhupgndPin, OUTPUT);
  analogWrite(PhdownlivePin, 0);                                                              // safty net disable all Pumps on stratup
  analogWrite(PhdowngndPin, 0);
  analogWrite(PhuplivePin, 0);
  analogWrite(PhupgndPin, 0);
  Serial.begin(115200);
  Serial.println("Initializing...");
  delay(1000);
    
}

void loop(void){

  GetPH();
  SetPH();
  
}

void SetPH(void)  {

  static unsigned long samplingTime = millis();
  if(millis() - samplingTime > SetPHrate ){

    if ( PHValue > configuredPH + PHdiff ){                                                   // Check if PH is ABOVE the configured and enable PH DOWN pump
             
        PHdowncount++;
        PhDownDelay += 1000;                                                                  // on each run add one second to pump
        PhUpDelay = PhUpDelayOriginal;                                                        // Reset counter on PH UP in case it went too high
          if ( PhDownDelay > PhDownMaxDelay ) {
            PhDownDelay -= 1000;                                                              // if above MAX seconds remove one second      
            PhUpDelay = PhUpDelayMin;                                                         // If Max hit - Reset oposite PH UP pump counter to Min - for safty                  
          }
        analogWrite(PhdownlivePin, 150);
        analogWrite(PhdowngndPin, 0);
        Serial.println("Ph Down");
        delay(PhDownDelay);                                                                   // HW delay to allow pumps to run clean
        analogWrite(PhdownlivePin, 0);
        analogWrite(PhdowngndPin, 0);
        Serial.println("Ph Down Stop");
      
    }
        if ( PHValue < configuredPH - PHdiff ){                                               // Check if PH is BELOW the configured and enable PH UP pump
          
            PHupcount++;
            PhUpDelay += 1000;                                                                // on each run add one second to pump
            PhDownDelay = PhDownDelayOriginal;                                                // Reset counter on PH DOWN in case it went too high
              if ( PhUpDelay > PhUpMaxDelay ) {
                PhUpDelay -= 1000;                                                            // if above MAX seconds remove one second   
                PhDownDelay = PhDownDelayMin;                                                // If Max hit - Reset oposite PH DOWN pump counter to Min - for safty
              }
            analogWrite(PhuplivePin, 150);
            analogWrite(PhupgndPin, 0);
            Serial.println("Ph UP");
            delay(PhUpDelay);                                                                  // HW delay to allow pumps to run clean
            analogWrite(PhuplivePin, 0);
            analogWrite(PhupgndPin, 0); 
            Serial.println("Ph Up Stop");
     
        }
          samplingTime=millis();                  
  }
}


void GetPH(void)  {
  
  static unsigned long samplingTime = millis();
  if(millis() - samplingTime > samplingInterval){

      pHArray[pHArrayIndex++] = analogRead(SensorPin);                                        // Collect PH array 
      long temp = analogRead(SensorPin);                                                      // debug raw analog value in int
      unsigned long temp2 = millis();                                                         // debug micro time         
      samplingTime=millis();                                                                  // reset timer
      
        if(pHArrayIndex == ArrayLenth){                                                       // check if array is full 

            voltage = (( avergearray(pHArray, ArrayLenth)* Vcc / 1024 ) + voltageoffset );    //function to calculate voltage 
            PHValue = PHoffset * voltage;                                                     //final result
            pHArrayIndex = 0;                                                                 //Rester Array Index
              
              Serial.print("time:");
              Serial.print(temp2);        
              Serial.print("\tArray:");
              Serial.print(temp);   
              Serial.print("\tVoltage:");
              Serial.print(voltage,3);
              Serial.print("\tpH value:");
              Serial.print(PHValue);
              Serial.print("\tConfigured PH:");
              Serial.print(configuredPH);
              Serial.print("\tPH DOWN timer:");
              Serial.print(PhDownDelay);
              Serial.print("\tPH UP timer:");
              Serial.print(PhUpDelay);
              Serial.print("\tPH UP run times:");
              Serial.print(PHupcount);
              Serial.print("\tPH Down run times:");
              Serial.println(PHdowncount);
                            
        }

  }
}



double avergearray(int* arr, int number) {                                                    // Avrage PH array 

  int i;
  double avg;
  long amount=0;
     
     for(i=0;i<number;i++){
      amount+=arr[i];

    }

      avg = amount/number;
      return avg;

}
