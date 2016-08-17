#include <SoftModem.h>

SoftModem modem = SoftModem();

void setup() {
	Serial.begin(115200);
	Serial.println("Booting");
	delay(100);
	modem.begin();
}

void loop() {
	while(modem.available()){
		int c = modem.read();
		if(isprint(c)){
			Serial.print((char)c);
		}
		else{
			Serial.print("(");
			Serial.print(c,HEX);
			Serial.println(")");      
		}
	}
	if(Serial.available()){
		modem.write(0xff);
		while(Serial.available()){
			char c = Serial.read();
			modem.write(c);
		}
	}
}
