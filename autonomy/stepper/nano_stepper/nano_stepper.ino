String str;

void setup()
{
  Serial.begin(115200);
  Serial.print("<Arduino Nano Ready>#");
  delay(500);
}

void loop()
{
  while(Serial.available() > 0)
  {
    str = Serial.readString();
    Serial.print(str + "#");
    
    if(str.equals("Hello world"))
    {
      Serial.print("Welcome home#");
      str = "";
    }
  }
}