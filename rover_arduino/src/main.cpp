#define BTN_CNT 4
const int BTN_PINS[] = {4, 4, 4, 4};

#define GRIPPER_CLOSED 0
#define GRIPPER_OPEN 90

#define SERVO_MA0 44
#define SERVO_MA1 45
#define SERVO_MY 46
#define SERVO_MG 5
#define SERVO_CP0 6
#define SERVO_CP1 7

#define TRIGGER_PIN  3
#define ECHO_PIN     2

#define STEPS_CENTER 0
#define STEPS_FULL 2200

#define BASE_MY 150
#define BASE_MA0 40
#define BASE_MA1 180
#define BASE_MG 0
#define BASE_CP 20
#define BASE_CP1 20

#include <Arduino.h>
#include <NewPing.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>
#include <ServoEasing.h>

#include <ros.h>

#include <nti_acs/ManipulatorArrow.h>
#include <nti_acs/CamPos.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/UInt8MultiArray.h>

using namespace std_msgs;
using namespace nti_acs;


Adafruit_NeoPixel strip = Adafruit_NeoPixel(24, 30, NEO_GRB + NEO_KHZ800);

NewPing sonar(TRIGGER_PIN, ECHO_PIN, 300);

AccelStepper stepper(AccelStepper::FULL4WIRE, 8, 9, 10, 11); 

ServoEasing man_yaw;
ServoEasing man_grip;
ServoEasing man_arrow_0;
ServoEasing man_arrow_1;
ServoEasing cam_yaw;
ServoEasing cam_pitch;
ServoEasing cam_pitch_1;

class NewHardware : public ArduinoHardware
{
  public:
  NewHardware():ArduinoHardware(&Serial1, 115200){};
};

ros::NodeHandle_<NewHardware>  nh;


void sub_cb_ma(const ManipulatorArrow& x){
    Serial.print(x.arrow0);
    Serial.println(x.arrow1);
    if (x.arrow0 != -1 && x.arrow0 != 255) man_arrow_0.easeTo(x.arrow0);
    if (x.arrow1 != -1 && x.arrow1 != 255) man_arrow_1.easeTo(x.arrow1);
}

void sub_cb_my(const Int16& x){
    Serial.println(x.data);
    if (x.data != -1) man_yaw.easeTo(x.data);
}

void sub_cb_mg(const Int16& x){
    if (x.data != -1) man_grip.easeTo(x.data);
}

void sub_cb_c(const CamPos& x){
    if (x.pitch != -1) {
        cam_pitch.easeTo(x.pitch);
        cam_pitch_1.easeTo(x.pitch);
    }
    if (x.yaw != -1) stepper.moveTo(STEPS_CENTER + x.yaw);
}

void sub_cb_s(const ColorRGBA& x){
    uint32_t c = strip.Color(x.r, x.g, x.b);
    for(uint16_t i = 0; i < strip.numPixels(); i++) strip.setPixelColor(i, c);
    strip.show();
}


ros::Subscriber<ManipulatorArrow> sub_ma("/ard/manip_arrow", sub_cb_ma);
ros::Subscriber<Int16> sub_mg("/ard/manip_grip", sub_cb_mg);
ros::Subscriber<Int16> sub_my("/ard/manip_yaw", sub_cb_my);

ros::Subscriber<CamPos> sub_c("/ard/cam_pos", sub_cb_c);
ros::Subscriber<ColorRGBA> sub_s("/ard/status", sub_cb_s);


UInt16 sonar_data;
ros::Publisher pub_sonar("/ard/sonar", &sonar_data);

UInt16 light_data;
ros::Publisher pub_light("/ard/light", &light_data);

UInt8MultiArray btn_data;
ros::Publisher pub_btn("/ard/btn", &btn_data);


long long int last_update = 0;


void calibrate_stepper() {
    stepper.setSpeed(50);
    while(!digitalRead(BTN_PINS[0])) stepper.runSpeed();
    stepper.stop();
    stepper.runToNewPosition(STEPS_CENTER);
}

void setup() {
    Serial.begin(9600);
    strip.begin();
    strip.setBrightness(50);
    strip.show();

    nh.initNode();
    nh.subscribe(sub_c);
    nh.subscribe(sub_ma);
    nh.subscribe(sub_mg);
    nh.subscribe(sub_my);
    nh.subscribe(sub_s);
    nh.advertise(pub_sonar);
    nh.advertise(pub_light);
    nh.advertise(pub_btn);

    for(int i = 0; i < BTN_CNT; i++) pinMode(BTN_PINS[i], INPUT);

    man_yaw.attach(SERVO_MY);
    man_grip.attach(SERVO_MG);
    man_arrow_0.attach(SERVO_MA0);
    man_arrow_1.attach(SERVO_MA1);
    cam_pitch.attach(SERVO_CP0);
    cam_pitch_1.attach(SERVO_CP1);

    man_yaw.easeTo(BASE_MY);
    man_grip.easeTo(BASE_MG);
    man_arrow_0.easeTo(BASE_MA0);
    man_arrow_1.easeTo(BASE_MA1);
    cam_pitch.easeTo(BASE_CP);
    cam_pitch_1.easeTo(BASE_CP1);

    stepper.setMaxSpeed(500);
    stepper.setSpeed(300);
    stepper.setAcceleration(200);

    memset(btn_data.data, 0, BTN_CNT);
    uint32_t c = strip.Color(255, 0, 0);
    for(uint16_t i = 0; i < strip.numPixels(); i++) strip.setPixelColor(i, c);
    strip.show();

    // man_grip.easeTo(0);
    // delay(1000);
    // man_grip.easeTo(40);
    // delay(1000);
    // man_grip.easeTo(80);
    // delay(1000);
    // man_grip.easeTo(120);
    // delay(1000);

    // stepper.setSpeed(50);
    // while(true) stepper.runSpeed();
    // calibrate_stepper();
    // stepper.moveTo(2200);
}

void loop() {
    // Serial.println(stepper.currentPosition());

    nh.spinOnce();

    stepper.run();
    delay(5);
    if ((millis() - last_update) > 200) {
        for(int i = 0; i < BTN_CNT; i++) btn_data.data[i] = digitalRead(BTN_PINS[i]);
        pub_btn.publish(&btn_data);

        light_data.data = analogRead(A0);
        pub_light.publish(&light_data);

        // sonar_data.data = sonar.ping_cm();
        // pub_sonar.publish(&sonar_data);

        last_update = millis();
    }
}