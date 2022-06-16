#include <Arduino.h>
#include <WiFi.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <ESP32Servo.h>
#include <math.h>
#include <string.h>
// #include <ESP_SBUS.h>  -> have not finished



#define M_PI  3.14159265358979323846f  /* pi */
#define stopStroke 0
#define downStroke 1
#define upStroke 2
#define glide 3

//Left Wing Configuration
#define upStrokeL 1900
#define downStrokeL 1400
#define midLeft 1600

//Right Wing Configuration
#define upStrokeR 1100
#define downStrokeR 1600
#define midRight 1400

Servo leftWing;
Servo rightWing;
Servo pitchTail;
Servo rollTail;



const int leftPin=21;
const int rightPin=19;
const int rollPin=18;
const int PitchPin=5;

bool flapMode;

const char* ssid     = "ntlab1802";
const char* password = "hoge1802";
// Set the rosserial socket server IP address
IPAddress server(192,168,30,243);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
// Make a chatter publisher
std_msgs::Int16 str_msg;
std_msgs::Int16 cnt_msg;

volatile double freqFlap=4;
volatile double pTail;
volatile double rTail;

bool failSafe;
bool lostFrame;

void freq_cb(const geometry_msgs::Twist& flap){
    freqFlap = flap.linear.x;
}

ros::Publisher chatter("chatter", &str_msg);
ros::Publisher cnt("cntr", &cnt_msg);
ros::Subscriber<geometry_msgs::Twist> flap("flapFreq", freq_cb);

// Be polite and say hello
char hello[13] = "hello world!";


volatile uint16_t counter=0;

volatile double getFlapMs(double freq){
  return (volatile double)(1000/freq);
}

volatile int16_t interpolateFlap(int amplitude, volatile double freq, int time){
    
    return (volatile int16_t) (amplitude * sin(((2*M_PI)/getFlapMs(freqFlap) * time)));

}



void setup()
{
  pinMode(2, OUTPUT);
  Serial.begin(115200);
  leftWing.attach(leftPin, 800, 2200);
  rightWing.attach(rightPin, 800, 2200);
  rollTail.attach(rollPin, 800, 2200);
  pitchTail.attach(PitchPin, 800, 2200);
  Serial.begin(115200);
  // x8r.begin();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  // Start to be polite
  nh.advertise(chatter);
  nh.subscribe(flap);
  flapMode=true;
}

void loop()
{

  // leftWing.writeMicroseconds(upStrokeL);
  // rightWing.writeMicroseconds(upStrokeR);
  // delay(100);
  // leftWing.writeMicroseconds(downStrokeL);
  // rightWing.writeMicroseconds(downStrokeR);
  // delay(100);
  if (nh.connected()) {
    digitalWrite(2, HIGH);
  
    if(flapMode==true){
      
      // str_msg.data = getFlapMs(freqFlap);
      volatile int16_t flapping = interpolateFlap(400, getFlapMs(freqFlap), counter);
      str_msg.data = flapping;
      chatter.publish( &str_msg );
      
      if(counter<getFlapMs(freqFlap))counter++;
      else counter=0;  

      if(millis()%20==0){
        leftWing.writeMicroseconds(midLeft + flapping);
        rightWing.writeMicroseconds(midRight - flapping); 
      }
      
    }
  }
  
  else {
    Serial.println("Not Connected");
    leftWing.writeMicroseconds(midLeft + 200);
    rightWing.writeMicroseconds(midRight - 200);
    digitalWrite(2, LOW);

  }

  nh.spinOnce();
  delay(1);
}