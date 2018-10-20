

/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Int16.h>

int forward = 10; 
int left =8;
int right =9;
int stop1 =11;
int a;
// ESC signal wire connected to pin 11 on arduino
int  val =100;

ros::NodeHandle  nh;

void messageCb( const std_msgs::Int16& msg)
{
      a=msg.data;
 
  
    if(a == 0)                                        // straight
 
 {

    nh.loginfo("tyhe val is ");
   analogWrite(forward, val);
   delay(10);
 
}
 if(a== 1)                                        // left

 {
   analogWrite(left, val);
   delay(10);
 }
if(a== 2)                                        // left

 {
   analogWrite(left, val);
   delay(10);
 }

 if(a== 3)                                        //right
 {
   analogWrite(right, val);
   delay(10);

}


 if(a== 4)                                        //right
 {
   analogWrite(right, val);
   delay(10);

}


}

ros::Subscriber<std_msgs::Int16> sub("chatter", &messageCb);

void setup()
{ 
  nh.initNode();
  nh.subscribe(sub);

  
}

void loop()
{
 
  nh.spinOnce();
  delay(1);
}

