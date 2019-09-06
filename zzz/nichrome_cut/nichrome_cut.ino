void setup()
{
    pinMode(2,OUTPUT);
    pinMode(3,OUTPUT); 
    delay(5000); 
}

void loop()
{
    digitalWrite(2,OUTPUT);
    delay(6000);
    digitalWrite(2,LOW);
    delay(100);
    digitalWrite(3,HIGH);
    delay(6000);
    digitalWrite(3,LOW);
    while(1){}
}
