#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

//#define SONAR_NUM 1          //The number of sensors. 
#define MAX_DISTANCE 200     //Mad distance to detect obstacles.
#define PING_INTERVAL 33     //Looping the pings after 33 microseconds.
#define PING_TIMEOUT 500

//unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
//unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
//uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

const int left_sensor = 2;
const int front_left_sensor = 4;
const int front_right_sensor = 6;
const int right_sensor = 8;

unsigned long _timerStart = 0;
int LOOPING = 40; //Loop for every 40 milliseconds.

ros::NodeHandle nh; //create an object which represents the ROS node.

sensor_msgs::Range range_left;
sensor_msgs::Range range_front_left;
sensor_msgs::Range range_front_right;
sensor_msgs::Range range_right;

ros::Publisher pub_range_left("ultrasound_left", &range_left);
ros::Publisher pub_range_front_left("ultrasound_front_left", &range_front_left);
ros::Publisher pub_range_front_right("ultrasound_front_right", &range_front_right);
ros::Publisher pub_range_right("ultrasound_right", &range_right);



void sendTrigger(int pin){

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);

}

void startTimer() {
  _timerStart = millis();
}

bool isTimeForLoop(int _mSec) {
  return (millis() - _timerStart) > _mSec;
}

void sensor_msg_init(sensor_msgs::Range &range_name, char *frame_id_name)
{
  range_name.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_name.header.frame_id = frame_id_name;
  range_name.field_of_view = 0.26;
  range_name.min_range = 0.2;
  range_name.max_range = 6.0;
}

unsigned long readEcho(int pin){

  long duration;
  pinMode(pin, INPUT);
  duration = pulseIn(pin, HIGH);
  return duration;
}

long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}

float microsecondsToMeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return (float)microsecondsToCentimeters(microseconds) / 100;
} 

void setup() {
  // initialize serial communication:
  //Serial.begin(115200);

  //pingTimer[0] = millis() + 75;
  //for (uint8_t i = 1; i < SONAR_NUM; i++)
  //  pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  
  nh.initNode();
  nh.advertise(pub_range_left);
  nh.advertise(pub_range_front_left);
  nh.advertise(pub_range_front_right);
  nh.advertise(pub_range_right);
  
  sensor_msg_init(range_left, "ultrasound_left");
  sensor_msg_init(range_front_left, "ultrasound_front_left");
  sensor_msg_init(range_front_right, "ultrasound_front_right");
  sensor_msg_init(range_right, "ultrasound_right");
  
  

}

void loop() {

  float left_m, front_left_m, front_right_m, right_m;
  
  if (isTimeForLoop(LOOPING)) {

    // get a reading from all the ultrasonic sensors
    sendTrigger(left_sensor);
    left_m = microsecondsToMeters( readEcho(left_sensor) );
    
    sendTrigger(front_left_sensor);
    front_left_m = microsecondsToMeters( readEcho(front_left_sensor) );
    
    sendTrigger(front_right_sensor);
    front_right_m = microsecondsToMeters( readEcho(front_right_sensor) );
    
    sendTrigger(right_sensor);
    right_m = microsecondsToMeters( readEcho(right_sensor) );
    

   /*
    Serial.print(cm);
    Serial.print("cm");  
    Serial.print(m);
    Serial.print("m");
    Serial.println();
   */

    range_left.range = left_m;
    range_left.header.stamp = nh.now();
    pub_range_left.publish(&range_left);

    range_front_left.range = front_left_m;
    range_front_left.header.stamp = nh.now();
    pub_range_front_left.publish(&range_front_left);

    range_front_right.range = front_right_m;
    range_front_right.header.stamp = nh.now();
    pub_range_front_right.publish(&range_front_right);

    range_right.range = right_m;
    range_right.header.stamp = nh.now();
    pub_range_right.publish(&range_right);

    
    startTimer();
  }
  nh.spinOnce();//Handle ROS events
 

  delay(100);
}
