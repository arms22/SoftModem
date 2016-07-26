SoftModem
====

Audio Jack Modem Library for Arduino, using FSK modulation with 1225 baud. Read this [blog post](http://translate.google.com/translate?js=y&prev=_t&hl=en&ie=UTF-8&layout=1&eotf=1&u=http%3A%2F%2Farms22.blog91.fc2.com%2Fblog-entry-350.html&sl=auto&tl=en) for a detailed explanation.


### Arduino Library Manager
Open the Arduino Library Manager and search for 'SoftModem'.

### Manual install
Create a folder 'SoftModem' inside your `libraries` folder and place these files there. 

### Use
This is an example sketch that forwards data to/from the serial port.

```Arduino
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
```

### Notes
SoftModem uses Timer2, therefore you can not make use of the `analogWrite()` function for  pins 3 and 11 in your sketch.

### Hardware
A shield is available at [Elechouse](http://www.elechouse.com/elechouse/index.php?main_page=product_info&cPath=90_92&products_id=2199). Of course you can build your own. Use this schematic:

![Schematic](http://www.elechouse.com/elechouse/images/product/softmodem/Arduino%20softmodem-5.jpg)

##License
BSD 3