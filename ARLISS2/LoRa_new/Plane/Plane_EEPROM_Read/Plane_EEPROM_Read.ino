#include <EEPROM.h>

void setup()
{
    Serial.begin(57600);
}

void loop()
{
    int EEPROM_Address;
    unsigned long SLEEP_TIME,TRAINING_TIME,STABILIZE_NOSEUP_TIME,STABILIZE_TIME,AUTO_TIME,DEEP_TIME;
    float Lat,Lon,Alt;

    for(EEPROM_Address = 0;EEPROM_Address < 6;++EEPROM_Address){
        Serial.println(EEPROM.read(EEPROM_Address));
    }

    EEPROM.get(EEPROM_Address,Lat);
    Serial.println(Lat);
    EEPROM_Address += 4;

    EEPROM.get(EEPROM_Address,Lon);
    Serial.println(Lon);
    EEPROM_Address += 4;

    EEPROM.get(EEPROM_Address,Alt);
    Serial.println(Alt);
    EEPROM_Address += 4;

    Serial.println(EEPROM.read(EEPROM_Address));
    ++EEPROM_Address;

    EEPROM.get(EEPROM_Address,SLEEP_TIME);
    Serial.println(SLEEP_TIME);
    EEPROM_Address += 4;

    EEPROM.get(EEPROM_Address,TRAINING_TIME);
    Serial.println(TRAINING_TIME);
    EEPROM_Address += 4;

    EEPROM.get(EEPROM_Address,STABILIZE_NOSEUP_TIME);
    Serial.println(STABILIZE_NOSEUP_TIME);
    EEPROM_Address += 4;

    EEPROM.get(EEPROM_Address,STABILIZE_TIME);
    Serial.println(STABILIZE_TIME);
    EEPROM_Address += 4;

    EEPROM.get(EEPROM_Address,AUTO_TIME);
    Serial.println(AUTO_TIME);
    EEPROM_Address += 4;

    EEPROM.get(EEPROM_Address,DEEP_TIME);
    Serial.println(DEEP_TIME);
    
    while(true){}
}
