/*
  Ultrasonic Sensor HC-SR04 ROS Publisher

*/


#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>


ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "sensor_info", &range_msg);

char frameid[] = "/ultrasound";

// defines pins numbers
const int trigPin = 11;
const int echoPin = 12;
// defines variables
long duration;
double distance_mm, distanceInch;

#define MAX_RANGE 1.00
#define MIN_RANGE 0.10

#define USE_ROS

void setup() {

#ifdef USE_ROS

  nh.initNode();
  nh.advertise(pub_range);

#else
  Serial.begin(115200); // Starts the serial communication
#endif
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

unsigned long publisher_timer;

void loop() {

  if ( millis() > publisher_timer ){

    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);

    // Calculating the distance
    //distance_mm = random(10, 200); //duration * 0.034 / 2;
    distance_mm = duration * 0.00034 / 2;
    //distanceInch = duration * 0.0133 / 2;

#ifdef USE_ROS

      
    range_msg.range = distance_mm;
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.field_of_view= 0.5;
    range_msg.min_range= MIN_RANGE;
    range_msg.max_range= MAX_RANGE;
    range_msg.range = range_msg.range > range_msg.max_range ? range_msg.max_range : range_msg.range;
    range_msg.range = range_msg.range < range_msg.min_range ? range_msg.min_range : range_msg.range;
    
    
    range_msg.header.stamp = nh.now();
    range_msg.header.frame_id = "distance_sensor_frame";
    pub_range.publish(&range_msg);
#else
    // Prints the distance on the Serial Monitor
    Serial.print("Distance (mm): ");
    Serial.println(distance_mm);

    //Serial.print("Distance (inch): ");
    //Serial.println(distanceInch);
#endif
    publisher_timer =  millis() + 100; // 0.1 sec

  }
  nh.spinOnce();
}
