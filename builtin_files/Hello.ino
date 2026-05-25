/*
参照元：https://share-lab.net/arduino-helloworld1
*/

void setup {
    Serial.begin(9600);
}

void loop {
    Serial.println("Hello, World!");
    delay(1000);
}