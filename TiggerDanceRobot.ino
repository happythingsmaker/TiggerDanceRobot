// http://HappyThingsMaker.com
// 행복물건개발자 박은찬
// 서보 드라이버를 이용해서 서보를 돌림
// 시리얼로 서보를 돌리라는 신호가 들어오면 서보를 들었다 내림


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

#define PIN_NUMBER_SW_SERIAL_TX     2
#define PIN_NUMBER_SW_SERIAL_RX     3
#define LONG_MAX 2147483000

class ServoDriver {
        int middle = 300; // 1500us     //45 degree
        int diff = 100;     // 100 이란 숫자가 500us 를 가져간다.
        int minAngle = middle - diff ; // 0 degree  // 1000us
        int maxAngle = middle + diff ;  // 90 degree    //2000ys
        Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);

        const int ANGLE_CENTER = 90;
        // 180도로 가게 하면 CCW 로 간다 하면 그 것은 CCW 모터


        const int CCW = 0;
        const int CW = 1;
        const int MOTOR_ROTATION_INFO[16] = {CCW, CW, CW, CCW, CCW};
        const int DIFF_MOVEMENT_ANGLE  = 45;

    public:
        setup() {
            pwm1.begin();
            pwm1.setPWMFreq(52); // This is the maximum PWM frequency
            // pwm1.setPWMFreq(50);  // This is the maximum PWM frequency
            // 50 hz = 20ms duration
            // 1-2ms duty cycle
        }

        void go(int motorNum, int angle) {
            // 언제나 180도 움직이는 모터라고 생각한다. 하지만 실상은 다를 것이다.
            //그래도 그 것이 직관적이다.
            // 센터 포지션은 항상 90으로 취급한다.
            pwm1.setPWM(motorNum, 0, map(angle, 0, 180, minAngle, maxAngle));
        }

        void goCenterPosition(int motorNum) {
            go(motorNum, ANGLE_CENTER);
        }

        void goUpPosition(int motorNum) {
            if (MOTOR_ROTATION_INFO[motorNum] == CCW) {
                go(motorNum, ANGLE_CENTER + DIFF_MOVEMENT_ANGLE);
            } else {
                go(motorNum, ANGLE_CENTER - DIFF_MOVEMENT_ANGLE);
            }
        }

        void goDownPosition(int motorNum) {
            if (MOTOR_ROTATION_INFO[motorNum] == CCW) {
                go(motorNum, ANGLE_CENTER - DIFF_MOVEMENT_ANGLE);
            } else {
                go(motorNum, ANGLE_CENTER + DIFF_MOVEMENT_ANGLE);
            }
        }


        void openPosition(int motorNum) {
            if (motorNum % 2 == 0) {
                go(motorNum, 30);
            } else {
                go(motorNum, 0);
            }
        }

        void closePosition(int motorNum) {
            if (motorNum % 2 == 0) {
                go(motorNum, 0);
            } else {
                go(motorNum, 30);
            }
        }

        void torqueOff() {
            for (int i = 0 ; i < 16 ; i ++) {
                pwm1.setPWM(i, 0, 0);

            }
        }

};

ServoDriver servoDriver;
SoftwareSerial BTSerial(PIN_NUMBER_SW_SERIAL_RX, PIN_NUMBER_SW_SERIAL_TX);
long lastMsMotorUp[12] = {0};
long lastReceivedTime = 0;

void setup() {
    servoDriver.setup();
    BTSerial.begin(9600);
    Serial.begin(9600);


}
long currentMillis;
bool isFirst[12] = {true};
bool switchFlag = false;
void loop() {
    currentMillis = millis();
    for (int i = 0 ; i < 12 ; i ++) {
        // 500ms 후에 자동으로 복귀하게 하는 로직
        if (currentMillis  > lastMsMotorUp[i] + 150) {
            if (isFirst[i]) {
                servoDriver.goCenterPosition(i);
                Serial.println("return");
                isFirst[i] = false;
            }
        }
    }

    if (currentMillis  - lastReceivedTime > 2000) {
        //torque off
        servoDriver.torqueOff();

    }

    char serialBuffer[10] = {0};

    BTSerial.listen();
    if (BTSerial.available()) {
        //BTSerial.readBytesUntil('\n', serialBuffer, 3);
        serialBuffer[0] = BTSerial.read();
        lastReceivedTime = millis();
        int tempMotorNumber = 0;

        if (serialBuffer[1] == 'd') {
            return;
        }
        switch (serialBuffer[0]) {
            case 'b':
                tempMotorNumber = 0;
                break;
            case 's':
                tempMotorNumber = 1;
                break;
            case 'r':
                tempMotorNumber = 1;
                break;
            case 'T':
                break;
            case 't':
                break;
            case 'f':
                break;
            case 'h':

                if (switchFlag) {
                    tempMotorNumber = 2;
                    switchFlag = false;

                } else {
                    tempMotorNumber = 3;
                    switchFlag = true;
                }
                servoDriver.goDownPosition(tempMotorNumber);
                break;
            case 'o':
                if (switchFlag) {
                    tempMotorNumber = 2;
                    switchFlag = false;

                } else {
                    tempMotorNumber = 3;
                    switchFlag = true;
                }
                servoDriver.goUpPosition(tempMotorNumber);
                break;
                break;
            case 'c':
                tempMotorNumber = 4;


                break;
            case 'y':

                break;
            case 'Y':

                break;
            case 'C':
                tempMotorNumber = 1;
                break;
            default:
                return;
                break;
        }

        if (tempMotorNumber == 0 || tempMotorNumber == 1 || tempMotorNumber == 4 ) {
            servoDriver.goUpPosition(tempMotorNumber);
        } else {
        }
        lastMsMotorUp[tempMotorNumber] = millis();
        isFirst[tempMotorNumber] = true;

    }

    if (Serial.available()) {
        int tempMotorNumber = Serial.read();
        lastReceivedTime = millis();

        tempMotorNumber -= 48;

        if (tempMotorNumber == 0 || tempMotorNumber == 1) {
            servoDriver.goUpPosition(tempMotorNumber);
        } else {
            if (random(2)) {
                servoDriver.goUpPosition(tempMotorNumber);
            } else {
                servoDriver.goDownPosition(tempMotorNumber);
            }
        }
        //servoDriver.goDownPosition(tempMotorNumber);
        lastMsMotorUp[tempMotorNumber] = millis();
        isFirst[tempMotorNumber] = true;
        Serial.println(tempMotorNumber);
    }
}
