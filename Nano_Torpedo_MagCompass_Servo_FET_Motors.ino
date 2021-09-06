// Outputs servo and PWM-FET-Controlled pair of motor signals to keep model torpedo in a straight line


#include <Servo.h>
#include <Wire.h>      // For talking I2C to the magnetometer
#define MAG_ADDRESS ((char) 0x1E) //I2C Address for The HMC5883 - nb - 400kbps max I2C data rate
#define errorval -4096
#define steeramp 2    // amplify the steering factor to try and return faster to a straight line 

Servo myservo;        // create servo object to control a servo
int pos;              // variable to store the servo position
int midships=102;     // servo trim offset for stright ahead 
int target=errorval;  // target heading to try and stay on

// We capture every reading, even if we don't use them all
#define max_buf 18      // X, Z, and Y readings of 2 bytes each - so a 18-sized buffer gives us room for 3 readings to accumulate if necessary - seems we never need more than 1 though...
int8_t mag_buf[max_buf];    // where the bytes are buffered - only index 0 is used - 1+ is usually wasted
int mag_buf_i=0;      // pointer to where we are upto - gets set to 0 after the 0 got used.
int16_t *mag_dat=(int16_t *)&mag_buf; // points to mag_buf
int8_t gain=B00000000;  // 0 means ± 0.88 Ga (max). B11100000 means ± 8.1 Ga (min)

long idlecount=0;     // Gives us an idea of how long we waited inbetween readings - if this is a very small number, code needs to be optimised.

long start_time = 0;
long previous_time = 0;
long loop_duration = 0;

volatile boolean isDataReady = false; // Gets set inside our interrupt when data is ready to read

int ledPin = 13;                 // digital pin 13 = LED
int tog=0;

void setup() {
  int i;
  Serial.begin(115200);
  Serial.println("Initializing Nano_Interrupt_Compass_Buffer ...");
  pinMode(ledPin, OUTPUT);      // sets the digital pin on the LED as output
  digitalWrite(ledPin, tog++&1);// toggle LED

  myservo.attach(5);  // attaches the servo on pin 9 to the servo object

  myservo.write(midships);              // center it
  delay(300);
  digitalWrite(ledPin, tog++&1);// toggle LED
  for (pos = midships; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  myservo.write(midships);              // center it
  delay(300);
  digitalWrite(ledPin, tog++&1);// toggle LED
  for (pos = midships; pos >= 0; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }

  
  Wire.begin();
  configMag();
  attachInterrupt(0, magDataReady, RISING); // INT0 is pin D2
  previous_time = micros();



  myservo.write(midships);              // center it


  
} // setup

void magDataReady() {     // interrupt function when mag DRDY pin is brought LOW
  isDataReady = true;
} // magDataReady interrupt

void loop() {
  start_time = micros();
  loop_duration = start_time - previous_time;
  idlecount++;

  if(isDataReady) {
      isDataReady = false;
      readMag();
  }
  if(mag_buf_i>0) {     // a full reading is waiting for us now
    int i=0;
    int newpos;
    int heading=180-atan2(mag_dat[i], mag_dat[i+2])/0.0174532925; // 0/359=N, 90=E, 180=S, 270=W 
    if(target==errorval) target=heading;
    newpos=target-heading; 
    if(newpos>180)newpos-=360;
    if(newpos<-180)newpos+=360;
    if((newpos*steeramp+midships)!=pos) { pos=newpos*steeramp+midships; if((pos>=0)&&(pos<=180))myservo.write(pos);}
    

    // Dump the buffer out to serial now
    Serial.print(newpos, DEC); Serial.print(" ");  // degrees
    Serial.print(heading, DEC); Serial.print(" ");  // degrees
    Serial.print(target, DEC); Serial.print(" ");  // degrees
    
    Serial.print(mag_dat[i++], DEC); Serial.print(" "); // X
    Serial.print(mag_dat[i++], DEC); Serial.print(" "); // Z
    Serial.print(mag_dat[i++], DEC); Serial.print(" "); // Y
    // Serial.print(mag_extra, DEC); Serial.print(" "); // # weird byte (status?) -1 always (commenting releases 12 iterations)
    // Serial.print(mag_buf_i, DEC); Serial.print(" "); // # buffered readings - always 6 - seems our full-speed and serial printing has spare time !
    // Serial.print(loop_duration, DEC); Serial.print(" ");// # microseconds we waited for a reading - around 5200
    Serial.print(idlecount, DEC);       // # iterations we waited
    Serial.println();
  digitalWrite(ledPin, tog++&1);// toggle LED

    mag_buf_i=0;
    previous_time = start_time;
    idlecount=0;
  }

} // loop


// read 6 bytes (x,y,z magnetic field measurements) from the magnetometer
void readMag() { 
  Wire.beginTransmission(MAG_ADDRESS);  // multibyte burst read of data registers (from 0x03 to 0x08)
  Wire.write((byte) 0x03);    // the address of the first data byte
  Wire.endTransmission();
  
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.requestFrom(MAG_ADDRESS, 6); // Request 6 bytes

  int i = 1;
  while(Wire.available())
  { 
    mag_buf[i+mag_buf_i++] = Wire.read();  // Read one byte - nb - order = X, Z, Y
    i=-i;       // to store bytes in opposite order
  }
  i=Wire.read();    // dunno what this is - needs it tho I think - always -1
  Wire.endTransmission();
  if(mag_buf_i>=max_buf-6)mag_buf_i=6;  // prevent memory leak
  
  // put the device back into single measurement mode - needs this.
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write((byte) 0x02);
  Wire.write((byte) 0x01);
  Wire.endTransmission();
  
} // readMag



void configMag() {
  uint8_t mag_name;
  
  // make sure that the device is connected
  Wire.beginTransmission(MAG_ADDRESS); 
  Wire.write((byte) 0x0A); // Identification Register A
  Wire.endTransmission();
  
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.requestFrom(MAG_ADDRESS, 1);
  mag_name = Wire.read();
  Wire.endTransmission();
  
  if(mag_name != 0x48) {
    Serial.println("HMC5883L not found!");
    Serial.print(mag_name, HEX); Serial.println(" found, should be 0x48");
    delay(1000);
  }
  
  // Register 0x00: CONFIG_A
  // normal measurement mode (0x00) and 75 Hz ODR (0x18)
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write((byte) 0x00);
  Wire.write((byte) 0x18);
  Wire.endTransmission();
  delay(5);
  
  // Register 0x01: CONFIG_B
  // default range of +/- 130 uT (0x20)
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write((byte) 0x01);
  //  Wire.write((byte) 0x20);
  Wire.write((byte) gain); // ± 0.88 Ga - B00000000 through B11100000 is max to min gain
//  Wire.write((byte) 0x00); // ± 0.88 Ga - B00000000 through B11100000 is max to min gain

  Wire.endTransmission();
  delay(5);
  
  // Register 0x02: MODE
  // continuous measurement mode at configured ODR (0x00)
  // possible to achieve 160 Hz by using single measurement mode (0x01) and DRDY
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write((byte) 0x02);
  Wire.write((byte) 0x01);
  Wire.endTransmission();
  
  delay(200);
}
