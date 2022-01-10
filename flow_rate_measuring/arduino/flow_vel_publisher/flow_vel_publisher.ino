#include <ros.h>
#include <sensor_msgs/Temperature.h>

// create node handling object
ros::NodeHandle nh;

// create ros message object
sensor_msgs::Temperature vel_msg;

// publisher
ros::Publisher vel_pub("flow_vel", &vel_msg);

// parameters for measuring flow rate
float V_0 = 5.0;
float rho = 1.204;
float offset = 0;
int offset_size = 1000;
int veloc_mean_size = 50; //debug
int zero_span = 0;

void setup() {
  //Serial.begin(9600);
  nh.initNode();
  nh.advertise(vel_pub);

  for (int i = 0; i < offset_size; ++i) {
    offset += analogRead(A0) - (1023.0/2.0);
  }
  offset /= offset_size;
}

void loop() {
  // read flow velocity
  float adc_avg = 0; float veloc = 0.0;

  for (int i = 0; i < veloc_mean_size; ++i) {
    adc_avg += analogRead(A0) - offset;
  }
  adc_avg /= veloc_mean_size;

  if (adc_avg > 512 - zero_span and adc_avg < 512 + zero_span) {
  }else {
    if (adc_avg < 511.5) {
      veloc = -sqrt((-10000.0*((adc_avg/1023.0)-0.5))/rho);
    } else {
      veloc = sqrt((10000.0*((adc_avg/1023.0)-0.5))/rho);
    }
  }
  
  vel_msg.header.stamp = nh.now();
  vel_msg.temperature = veloc;
  vel_pub.publish(&vel_msg);
  nh.spinOnce();
  //Serial.println(veloc);
  //delay(10);
}
