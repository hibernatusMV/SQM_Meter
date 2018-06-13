#include <Wire.h> 
#include <I2C_Anything.h> // Library to split value of a specific datatype into bytes
#include <FreqCounter.h>

float         frequency;
int           period = 100;
unsigned long frq  = 1L;

boolean measure = false;


void setup() {
  Serial.begin(9600);        // connect to the serial port
  Serial.println(F("Frequency Counter started ..."));
  
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void requestEvent(void) {
  // send value to I2C and split value according to respective datatype into bytes
  I2C_writeAnything(frequency);
}

void receiveEvent(int numBytes) {
  if ( numBytes != 2 ) {
      while ( Wire.available() ) {
        Wire.read();
    }
    return;
  }

  if ( Wire.available() ) {
    period = Wire.read();

    if ( Wire.available() ) {
      // read received value as bytes and combine to original datatype
      period = period + ( Wire.read() << 8 );
      //Serial.print(F("Period: "));
      //Serial.println(period);
      measure = true;
    }
  }
}

void loop() {
  if ( measure ) {
    // wait if any serial is going on
    FreqCounter::f_comp=0;   // Cal Value / Calibrate with professional Freq Counter
    FreqCounter::start(period);  // 100 ms Gate Time
  
    while (FreqCounter::f_ready == 0) { 
      frq=FreqCounter::f_freq;
    }
    
    frequency = (frq * 1000) / period;
    
    // if cannot measure the number of pulses set to freq 1hz which is mag 21.95
    if ( frequency < 1 )
      frequency = 1;
    Serial.print(F("Frequency: "));
    Serial.println(frequency);
    
    measure = false;
  }
}
