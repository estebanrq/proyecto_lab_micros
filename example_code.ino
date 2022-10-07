int duration = 0;
long distance = 0;
int firstduration = 0;
long firstdistance = 0;
int secondduration = 0;
long seconddistance = 0;
#include <Servo.h>
Servo myservo;
int pos = 0;
void setup()
{
    pinMode(5, OUTPUT);
    pinMode(6, INPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    myservo.attach(3);
    Serial.begin(9600);
}
void loop()
{
    digitalWrite(5, HIGH);
    delayMicroseconds(10);
    digitalWrite(5, LOW);
    delayMicroseconds(2);
    duration = pulseIn(6, HIGH);
    delay(100);
    distance = duration * 0.034 / 2;
    Serial.println(distance);
    if (distance <= 20)
    {
        digitalWrite(7, LOW);
        digitalWrite(8, LOW);
        digitalWrite(9, LOW);
        digitalWrite(10, LOW);
        delay(300);
        digitalWrite(7, LOW);
        digitalWrite(8, HIGH);
        digitalWrite(9, LOW);
        digitalWrite(10, HIGH);
        delay(350);
        digitalWrite(7, LOW);
        digitalWrite(8, LOW);
        digitalWrite(9, LOW);
        digitalWrite(10, LOW);
        myservo.write(0);
        delay(500);
        digitalWrite(5, HIGH);
        delayMicroseconds(10);
        digitalWrite(5, LOW);
        delayMicroseconds(2);
        firstduration = pulseIn(6, HIGH);
        delay(100);
        firstdistance = firstduration * 0.034 / 2;
        int first = firstdistance;
        Serial.println(firstdistance);
        myservo.write(90);
        delay(500);
        myservo.write(180);
        delay(500);
        digitalWrite(5, HIGH);
        delayMicroseconds(10);
        digitalWrite(5, LOW);
        delayMicroseconds(2);
        secondduration = pulseIn(6, HIGH);
        delay(100);
        seconddistance = secondduration * 0.034 / 2;
        int second = seconddistance;
        Serial.println(seconddistance);
        myservo.write(90);
        delay(500);
        if (first < second)
        {
            digitalWrite(7, LOW);
            digitalWrite(8, HIGH);
            digitalWrite(9, HIGH);
            digitalWrite(10, LOW);
            delay(500);
        }
        else if (first > second)
        {
            digitalWrite(7, HIGH);
            digitalWrite(8, LOW);
            digitalWrite(9, LOW);
            digitalWrite(10, HIGH);
            delay(500);
        }
    }
    else
    {
        digitalWrite(7, HIGH);
        digitalWrite(8, LOW);
        digitalWrite(9, HIGH);
        digitalWrite(10, LOW);
    }
}