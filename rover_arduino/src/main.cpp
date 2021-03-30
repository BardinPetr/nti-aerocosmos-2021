#include <Arduino.h>
#include <Servo.h>

#include <ros.h>

using namespace std_msgs;
using namespace nti_acs;


Servo man_yaw;
Servo man_grip;
Servo man_arrow_0;
Servo man_arrow_1;
Servo cam_yaw;
Servo cam_pitch;

class NewHardware : public ArduinoHardware
{
  public:
  NewHardware():ArduinoHardware(&Serial1, 115200){};
};

ros::NodeHandle_<NewHardware>  nh;

void sub_cb_m( const nti_acs::Manipulator& x){
    if (x.arrow0 != -1 && x.arrow0 != 252) man_arrow_0.write(x.arrow0);
    if (x.arrow1 != -1 && x.arrow1 != 252) man_arrow_1.write(x.arrow1);
    if (x.yaw != -1) man_yaw.write(x.yaw);
    if (x.grip != -1) man_grip.write(x.grip ? GRIPPER_CLOSED : GRIPPER_OPEN);
}
ros::Subscriber<Manipulator> sub_m("/ard/manipulator", sub_cb_m);


void sub_cb_c( const nti_acs::CamPos& x){
    if (x.pitch != -1 && x.pitch != 252) cam_pitch.write(x.pitch);
    if (x.yaw != -1 && x.yaw != 252) {
        // TODO
    }
}
ros::Subscriber<CamPos> sub_c("/ard/cam_pos", sub_cb_c);

UInt16 sonar_data;
ros::Publisher<UInt16> pub_sonar("/ard/sonar", &sonar_data);


void setup() {
    nh.initNode();
    nh.subscribe(sub_c);
    nh.subscribe(sub_m);

    man_yaw.attach();
    man_grip.attach();
    man_arrow_0.attach();
    man_arrow_1.attach();
    cam_yaw.attach();
    cam_pitch.attach();
}

void loop() {
    sonar_data.data = 123;
    pub_sonar.publish(&sonar_data);

    nh.spinOnce();
    delay(1);
}