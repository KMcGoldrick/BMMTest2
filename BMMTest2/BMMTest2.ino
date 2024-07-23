#include <bmm150_defs.h>
#include <bmm150.h>
int loopCnt;
int8_t rslt;
float x, y, z;
bmm150_dev MyBMM150;

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

    rslt = bmm150_init(&MyBMM150);
    Serial.print("bmm150_init: ");Serial.println(rslt);

    while (true) {}
}

void loop() {
        ++loopCnt;
        serialData();
}
