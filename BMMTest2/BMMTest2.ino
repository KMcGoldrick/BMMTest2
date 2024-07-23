#include <bmm150_defs.h>
#include <bmm150.h>
int loopCnt;
float x, y, z;

void serialData() {
    Serial.print(0);
    Serial.print(',');
    Serial.print(loopCnt);
    Serial.print(',');
    Serial.print(x);
    Serial.print(',');
    Serial.print(y);
    Serial.print(',');
    Serial.println(z);
}

void setup() {
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Started");
}

void loop() {
        ++loopCnt;
        serialData();
}
