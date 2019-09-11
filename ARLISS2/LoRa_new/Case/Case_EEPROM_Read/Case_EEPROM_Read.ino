#include <EEPROM.h>

void setup()
{
    Serial.begin(57600);
}

void loop()
{
    int EEPROM_Address;
    int Voltage_measure_value;
    int default_Voltage_measure_value;
    unsigned long time_cds,time_height,time_nichrome1_start,time_nichrome1_end,time_nichrome2_start;
    float height,lat,lon,alt,pressure,temp,humidity;
    for(EEPROM_Address = 0;EEPROM_Address < 2;++EEPROM_Address){
        Serial.println(EEPROM.read(EEPROM_Address));
    }
    EEPROM.get(EEPROM_Address,Voltage_measure_value);
    Serial.println(Voltage_measure_value);
    EEPROM_Address += 2;
    EEPROM.get(EEPROM_Address,time_cds);
    Serial.println(time_cds);
    EEPROM_Address += 4;
    EEPROM.get(EEPROM_Address,height);
    Serial.println(height);
    EEPROM_Address += 4;
    EEPROM.get(EEPROM_Address,time_height);
    Serial.println(time_height);
    EEPROM_Address += 4;
    EEPROM.get(EEPROM_Address,time_nichrome1_start);
    Serial.println(time_nichrome1_start);
    EEPROM_Address += 4;
    EEPROM.get(EEPROM_Address,time_nichrome1_end);
    Serial.println(time_nichrome1_end);
    EEPROM_Address += 4;
    EEPROM.get(EEPROM_Address,time_nichrome2_start);
    Serial.println(time_nichrome2_start);
    EEPROM_Address += 4;
    EEPROM.get(EEPROM_Address,lat);
    Serial.println(lat);
    EEPROM_Address += 4;
    EEPROM.get(EEPROM_Address,lon);
    Serial.println(lon);
    EEPROM_Address += 4;
    EEPROM.get(EEPROM_Address,alt);
    Serial.println(alt);
    EEPROM_Address += 4;
    EEPROM.get(EEPROM_Address,pressure);
    Serial.println(pressure);
    EEPROM_Address += 4;
    EEPROM.get(EEPROM_Address,temp);
    Serial.println(temp);
    EEPROM_Address += 4;
    EEPROM.get(EEPROM_Address,humidity);
    Serial.println(humidity);
    EEPROM_Address += 4;
    EEPROM.get(EEPROM_Address,default_Voltage_measure_value);
    Serial.println(default_Voltage_measure_value);
    while(true){}
}
