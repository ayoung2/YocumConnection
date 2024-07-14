#include <Keypad.h> 
#include <EEPROM.h>
#include <SoftwareSerial.h>

#define NOT_USED_0       0
#define NOT_USED_1       1
#define KEYPAD_ROW_1     2
#define KEYPAD_ROW_2     3
#define KEYPAD_ROW_3     4
#define KEYPAD_ROW_4     5
#define KEYPAD_COL_1     6
#define KEYPAD_COL_2     7
#define KEYPAD_COL_3     8
#define SFX_TX           9
#define SFX_RX           10
#define SIGNAL_RED_PIN   11
#define SIGNAL_YLW_PIN   12
#define SIGNAL_GRN_PIN   13
#define SWITCH_PIN_OUT   14 //A0 - POS OUT
#define SWITCH_PIN_IN    15 //A1 - POS IN
#define DISPLAY_CS       16 //A2
#define DISPLAY_SDI      17 //A3
#define DISPLAY_SCL      18 //A4
#define SFX_RST          19 //A5        

String crLeg = "#258533*";
String saLeg = "#258511*";
String currentInput = "";
char numstr[21];
 
bool switchNormal = true;

int volume = 255;
int timeout = 600;
int settingVolume = 0;
int settingTimeout = 0;
int iaisRadioOffset = 5;
int state = 0;
 
const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns

char keys[ROWS][COLS] = 
{
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

byte rowPins[ROWS] = {KEYPAD_ROW_1, KEYPAD_ROW_2, KEYPAD_ROW_3, KEYPAD_ROW_4};
byte colPins[COLS] = {KEYPAD_COL_1, KEYPAD_COL_2, KEYPAD_COL_3};

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

unsigned long delayTime = 10000; // 10 sec
unsigned long delayStart = 0; // the time the delay started
bool delayRunning = false; // true if still waiting for delay to finish

unsigned long buttonHoldTime = 0;
unsigned long buttonHoldDelay = 2000;
int buttonPressed = -1;

unsigned long delayResetScreenTime = 0;
unsigned long delayResetScreenDelay = 60000;

bool debugThroughComp = false;
SoftwareSerial ss = SoftwareSerial(SFX_TX, SFX_RX);

#define LINE_BUFFER_SIZE 80 //!< Size of the line buffer
char line_buffer[LINE_BUFFER_SIZE];

void serialPrintLine(String string)
{
  if(debugThroughComp)
  {
    Serial.println(string);
  }
}

#pragma region display
void display_end_data_transmission()
{
  digitalWrite(DISPLAY_CS, HIGH);  // CS = 1;
  delay(1);
}

void display_data(char i)
{    
  unsigned int m;
  for(m=0;m<8;m++){        // Clock in DB0 - DB7
    if((i&0x80)==0x80)
    digitalWrite(DISPLAY_SDI, HIGH);
    else
    digitalWrite(DISPLAY_SDI, LOW);
    while(0);
    i=(i<<1);
    digitalWrite(DISPLAY_SCL, LOW);
    while(0);
    digitalWrite(DISPLAY_SCL, HIGH);
    while(0);
    digitalWrite(DISPLAY_SCL, LOW);
    }
}

void display_start_data_transmission()
{  
  digitalWrite(DISPLAY_CS, LOW);   // CS = 0;
  delay(1);
  digitalWrite(DISPLAY_SDI, HIGH); // RS = 1;
  digitalWrite(DISPLAY_SCL, LOW);
  digitalWrite(DISPLAY_SCL, HIGH); // Clock RS Pin in
  digitalWrite(DISPLAY_SCL, LOW);
  delay(1);
  digitalWrite(DISPLAY_SDI, LOW);  // RW = 0;
  digitalWrite(DISPLAY_SCL, LOW);
  digitalWrite(DISPLAY_SCL, HIGH); // Clock RW Pin in
  digitalWrite(DISPLAY_SCL, LOW);
}

void display_command(char i)
{  
  unsigned int m;
  digitalWrite(DISPLAY_CS, LOW);   // CS = 0;
  digitalWrite(DISPLAY_SDI, LOW);  // RS = 0;
  digitalWrite(DISPLAY_SCL, LOW);
  digitalWrite(DISPLAY_SCL, HIGH); // Clock RS Pin in
  digitalWrite(DISPLAY_SCL, LOW);
  delay(1);
  digitalWrite(DISPLAY_SDI, LOW);  // RW = 0;
  digitalWrite(DISPLAY_SCL, LOW);
  digitalWrite(DISPLAY_SCL, HIGH); // Clock RW Pin in
  digitalWrite(DISPLAY_SCL, LOW);
  for(m=0;m<8;m++){        // Clock in DB0 - DB7
    if((i&0x80)==0x80)
    digitalWrite(DISPLAY_SDI, HIGH);
    else
    digitalWrite(DISPLAY_SDI, LOW);
    while(0);
    i=(i<<1);
    digitalWrite(DISPLAY_SCL, LOW);
    while(0);
    digitalWrite(DISPLAY_SCL, HIGH);
    while(0);
    digitalWrite(DISPLAY_SCL, LOW);
    }
  digitalWrite(DISPLAY_CS, HIGH);  // CS = 1;
}

void display_clear_screen(){         // clear display 
 display_command(0x01);
}

void display_return_home(){             // Return to home position
 display_command(0x02); 
}

void setDisplay(String firstLine, String secondLine, int firstLineOffset)
{
  char charFirstBuf[20];
  firstLine.toCharArray(charFirstBuf, 20);

  char charSecondBuf[20];
  secondLine.toCharArray(charSecondBuf, 20);

  display_clear_screen();
  display_return_home();                 // First Line
  display_start_data_transmission();
  for(int i=0;i<20 && i<(firstLine.length()+firstLineOffset);i++)
  {
    if(i < firstLineOffset)
    {
      display_data(' ');
    }
    else
    {
      display_data(charFirstBuf[i - firstLineOffset]);
    }
  }

  if(firstLine == "McNab Radio")
  {
    display_data(220);
  }
  
  display_end_data_transmission();
/******************************************/    
  display_command(0xc0);              // Second Line
  display_start_data_transmission();
  for (int i=0;i<20&&i<secondLine.length();i++)
  {
    display_data(charSecondBuf[i]);
  }
  display_end_data_transmission();

  serialPrintLine("");
  serialPrintLine("");
  serialPrintLine(firstLine);
  serialPrintLine(secondLine);
}

void testDisplay(String firstLine, String secondLine, int firstLineOffset)
{
  char charFirstBuf[20];
  firstLine.toCharArray(charFirstBuf, 20);

  char charSecondBuf[20];
  secondLine.toCharArray(charSecondBuf, 20);

  for(int i=0; i<1000; i+=40)
  {
    Serial.println(i);
    display_clear_screen();
    display_return_home();                 // First Line
    display_start_data_transmission();
    for(int j=i;j<i+20;j++)
    {
      display_data(j);
    }
    display_end_data_transmission();
  /******************************************/    
    display_command(0xc0);              // Second Line
    display_start_data_transmission();
    for (int j=i+20;j<i+40;j++)
    {
      display_data(j);
    }
    display_end_data_transmission();
    delay(5000);
  }
  
}

void display_init(){
  delay(300);

  //was 28 but set to 29 to get Western European Font Table 1
  display_command(0x29);             //Function set
  delay(2);
  display_command(0x08);             //Display OFF
  delay(2);
  display_command(0x01);             //Clear Display
  delay(2);
  display_command(0x06);             //Entry Mode set
  delay(2);
  display_command(0x02);             //Return Home
  delay(2);
  display_command(0x0C);             //Display ON
  delay(2);
}
#pragma endregion

#pragma region audio
void playDTMF(char digit) 
{
  ss.println("q");
  delay(20);
  //play sound - find the filename
  String filename = "P";
  
  if (digit == '*')
  {
    filename += "STAR";
  }
  else
  {
    filename += String(digit) + "   ";
  }

  filename += "    WAV";

  serialPrintLine(filename);
  ss.println(filename);
}

int readLine(void) {
  int x = ss.readBytesUntil('\n', line_buffer, LINE_BUFFER_SIZE);
  line_buffer[x] = 0;

  if (ss.peek() == '\r')
    ss.read();
  // stream->readBytesUntil('\r', line_buffer, LINE_BUFFER_SIZE);
  return x;
}

uint8_t volUp() {
  while (ss.available())
    ss.read();

  ss.println("+");
  readLine();

  uint8_t v = atoi(line_buffer);
  
  if(debugThroughComp)
  {
    Serial.println(v);
  }
  return v;
}

uint8_t volDown() {
  while (ss.available())
    ss.read();

  ss.println("-");
  readLine();

  uint8_t v = atoi(line_buffer);
  if(debugThroughComp)
  {
    Serial.println(v);
  }
  return v;
}

bool setVolume(int vol)
{
  int currentVol = volDown()/2;
  int tryCount = 0;
  serialPrintLine("Current Vol:" + String(currentVol) + ", Board Vol:" + String(volume));
  while(currentVol > volume && tryCount < 10)
  {
    currentVol = volDown()/2;
    serialPrintLine("Current Vol:" + String(currentVol) + ", Board Vol:" + String(volume));
  }
  
  while(currentVol < volume && tryCount < 10)
  {
    currentVol = volUp()/2;
    serialPrintLine("Current Vol:" + String(currentVol) + ", Board Vol:" + String(volume));
    if (currentVol == 0)
    {
      tryCount++;
    }
    else
    {
      tryCount = 0;
    }
  }

  return tryCount < 10;
}

/*!
 * @brief Do a hard reset by bringing the RST pin low
 * then read out the output lines
 * @return Returns the output lines
 */
boolean reset(void) {
  digitalWrite(SFX_RST, LOW);
  pinMode(SFX_RST, OUTPUT);
  delay(10);
  pinMode(SFX_RST, INPUT);
  delay(1000); // give a bit of time to 'boot up'

  // eat new line
  readLine();
  serialPrintLine(line_buffer); // Date and name

  readLine();
  // "Adafruit FX Sound Board 9/10/14"
  serialPrintLine(line_buffer); // Date and name
  if (!strstr(line_buffer, "Adafruit FX Sound Board"))
    return false;

  delay(250);

  readLine();
  // Serial.print("3>"); Serial.println(line_buffer);   // FAT type
  readLine();
  // Serial.print("4>"); Serial.println(line_buffer);   // # of files

  return true;
}

void flushInput() {
  // Read all available serial input to flush pending data.
  uint16_t timeoutloop = 0;
  while (timeoutloop++ < 40) {
    while(ss.available()) {
      ss.read();
      timeoutloop = 0;  // If char was received reset the timer
    }
    delay(1);
  }
}
#pragma endregion

bool moveSwitch(bool normal)
{
  if (normal != switchNormal)
  {
    if (normal)
    {
      //switch the switch to NORMAL
      analogWrite(SWITCH_PIN_OUT, 255);
    }
    else
    {
      //switch the switch to REVERSE
      analogWrite(SWITCH_PIN_OUT, 0);
    }

    EEPROM.write(12, normal);
    delay(1000);
  }

  bool incorrectPosition = false;
  for(int i = 0; i < 100; i++)
  {
    int variable = analogRead(SWITCH_PIN_IN);
    if(debugThroughComp)
    {
      Serial.print("Position: ");
      Serial.println(variable);
    }

    if (!normal && variable < 10)
    {
      incorrectPosition = true;
      break;
    }
    else if (normal && variable > 200)
    {
      incorrectPosition = true;
      break;
    }

    delay(50);
  }
  
  //switch is in position!
  switchNormal = normal;
  return incorrectPosition;
}

void setSignalStatus(int sigStatus, bool moveSwitch)
{
   digitalWrite(SIGNAL_YLW_PIN, HIGH); 
   digitalWrite(SIGNAL_RED_PIN, HIGH); 
   digitalWrite(SIGNAL_GRN_PIN, HIGH);
   
   switch(sigStatus)
   {
     case 0:
        digitalWrite(SIGNAL_RED_PIN, LOW); 
        serialPrintLine("Signal went Red");
        //do not move switch
        break;
      case 1:
        digitalWrite(SIGNAL_YLW_PIN, LOW);
        serialPrintLine("Signal went Yellow");
        //move the switch to reverse
        break;
      case 2:
        digitalWrite(SIGNAL_GRN_PIN, LOW);
        serialPrintLine("Signal went Green");
        //move the switch to normal
        break;
   }
}

void keypadEvent(KeypadEvent eKey)
{
  switch (keypad.getState())
  {
    case PRESSED:
      if(debugThroughComp)
      {
        serialPrintLine("Pressed: ");
        Serial.println(eKey);
  
        serialPrintLine("State: ");
        Serial.println(state);
      }

      playDTMF(eKey);

      if (state == 0)
      {
          if (eKey == '*' && currentInput == "")
          {
            if(debugThroughComp)
            {
              serialPrintLine("Entered Diag");
              Serial.print("Timeout: ");
              Serial.println(timeout);
              Serial.print("Volume: ");
              Serial.println(volume);
            }
            //Go into diagnostics - set volume and timeout period? - Can that data be stored even through power cycle?
            //EEPROM.write(0, 12);
            //Start with the volume
            //"SETTINGS"
            //"VOL(max 100):100"

            setDisplay("SETTINGS", "VOL(max 100):" + String(volume), 0);
            state = 1;
            settingVolume = volume;
          }
          else if (crLeg.startsWith(currentInput + eKey) || saLeg.startsWith(currentInput + eKey))
          {
            serialPrintLine("Valid Key!");
            //currentInput plus the current key match the start of one of the password strings
            currentInput += eKey;
            
            //Display the latest input to the user
            setDisplay("IAIS Radio", currentInput, iaisRadioOffset);
            
            //small delay for the tone and to show the user entered values
            delay(100);
            delayResetScreenTime = millis();
            
            if (crLeg == currentInput)
            {
               //set the display to showing the movement
               setDisplay("IAIS Radio", "Switch Reverse", iaisRadioOffset);
               currentInput = "";
               //set the red LED to on and the rest off
               setSignalStatus(0, true);
               //move the switch machine
               bool moved = moveSwitch(false);

               if(!moved)
               {
                  setDisplay("IAIS Radio", "SWITCH ERROR", iaisRadioOffset);
                  delay(5000);
               }
               //set the yellow LED to on and the rest off
               setSignalStatus(1, true);
               
               //Play the detector response sound - Switch is reverse
               playDTMF('R');
               
               //delay the input period for a few seconds
               delay(10000);
               
               setDisplay("IAIS Radio", "", iaisRadioOffset);

               // start delay
               delayTime = (long)timeout * 1000;
               delayStart = millis();
               delayRunning = true;

              if(debugThroughComp)
              {
                Serial.println("Setting delay information");
                Serial.println(delayTime);
                Serial.println(millis());
              }
            }
            else if (saLeg == currentInput)
            {
               serialPrintLine("Switch is Normal");
               //set the display to showing the movement
               setDisplay("IAIS Radio", "Switch Normal", iaisRadioOffset);
               
               currentInput = "";
               //set the red LED to on and the rest off
               setSignalStatus(0, true);
               //move the switch machine
               bool moved = moveSwitch(true);

               if(!moved)
               {
                  setDisplay("IAIS Radio", "SWITCH ERROR", iaisRadioOffset);
                  delay(5000);
               }
               //set the green LED to on and the rest off
               setSignalStatus(2, true);
               
               //Play the detector response sound - Switch is normal
               playDTMF('N');
               
               //delay the input period for a few seconds
               delay(10000);
               
               setDisplay("IAIS Radio", "", iaisRadioOffset);

               // start delay
               //Serial.println(timeout);
               delayTime = (long)timeout * 1000;
               delayStart = millis();
               delayRunning = true;

              if(debugThroughComp)
              {
                Serial.println("Setting delay information");
                Serial.println(delayTime);
                Serial.println(millis());
              }
            }
          }
          else if (currentInput != "" && (crLeg.startsWith(currentInput) || saLeg.startsWith(currentInput)))
          {
             bool foundBoard = reset();
             serialPrintLine("Found Board: " + String(foundBoard));
  
             serialPrintLine("Invalid Key");
             //invalid key - reset variable
             currentInput = "";
             
             //prompt the user it was invalid
             setDisplay("IAIS Radio", "Invalid Input", iaisRadioOffset);

             delay(1000);
               
             setDisplay("IAIS Radio", "", iaisRadioOffset);
          }

          break;
      }
      else if (state == 1)
      {
         if (eKey == '*')
         {
            if (settingVolume != volume)
            {        
                //Save - Yes/No?
                setDisplay("SETTINGS-VOLUME", "SAVE?YES=#/NO=*", 0);
                state = 2;
            }
            else
            {
                setDisplay("SETTINGS", "Timeout(sec):" + String(timeout), 0);
                settingTimeout = timeout; 
                state = 3;
            }
         }
         else if (settingVolume == 0 && eKey == '0')
         {
            //do nothing
         }
         else if (settingVolume > 0 && eKey == '#')
         {
            //save the value if available
            volume = settingVolume;
            EEPROM.write(0, volume);
            setVolume(volume);
            playDTMF(eKey);
            //Display SAVED
            setDisplay("SETTINGS-SAVED", "VOL(max 100):" + String(volume), 0);
         }
         else
         {
            int intKey = eKey - '0';
            if (settingVolume * 10 + intKey > 100)
            {
               settingVolume = intKey;
               setDisplay("SETTINGS", "VOL(max 100):" + String(settingVolume), 0);
            }
            else
            {
               settingVolume = settingVolume * 10 + intKey;
               setDisplay("SETTINGS", "VOL(max 100):" + String(settingVolume), 0);
            }
         }
      }
      else if (state == 2)
      {
          if (eKey == '#' || eKey == '*')
          {
              if (eKey == '#')
              {
                 volume = settingVolume;
                 EEPROM.write(0, volume);
                 setVolume(volume);
                 playDTMF(eKey);
                 setDisplay("SETTINGS", "VOLUME SAVED", 0);
                 delay(1000);
              }
              
              state = 3;
              setDisplay("SETTINGS", "Timeout(sec):" + String(timeout), 0);
              settingTimeout = timeout; 
          }
      }
      else if (state == 3)
      {
        if (eKey == '*')
         {
            if (settingTimeout != timeout)
            {        
                serialPrintLine("Setting Timeout: " + String(settingTimeout) + ", Timeout: " + String(timeout));
                //Save - Yes/No?
                setDisplay("SETTINGS-TIMEOUT", "SAVE?YES=#/NO=*", 0);
                state = 4;
            }
            else
            {
                state = 0;
                setDisplay("IAIS Radio", "", iaisRadioOffset);
            }
         }
         else if (settingTimeout == 0 && eKey == '0')
         {
            //do nothing
         }
         else if (settingTimeout > 0 && eKey == '#')
         {
            //save the value if available
            timeout = settingTimeout;
            float f = 0.00f;
            f += settingTimeout;
            EEPROM.put(6, (float)f);
            //Display SAVED
            setDisplay("SETTINGS-SAVED", "Timeout(sec):" + String(timeout), 0);
         }
         else
         {
            int intKey = eKey - '0';
            if (settingTimeout * 10 + intKey > 999)
            {
               settingTimeout = intKey;
               setDisplay("SETTINGS", "Timeout(sec):" + String(settingTimeout), 0);
            }
            else
            {
               settingTimeout = settingTimeout * 10 + intKey;
               setDisplay("SETTINGS", "Timeout(sec):" + String(settingTimeout), 0);
            }
         }
      }
      else if (state == 4)
      {
          if (eKey == '#' || eKey == '*')
          {
              if (eKey == '#')
              {
                 timeout = settingTimeout;
                 float f = 0.00f;
                 f += settingTimeout;
                 EEPROM.put(6, (float)f);
                 setDisplay("TIMEOUT SAVED", "", 0);
                 delay(1000);
              }
              
              state = 0;
              setDisplay("IAIS Radio", "", iaisRadioOffset);
          }
      }
      break;

    case HOLD:
        if(state == 0 && (eKey == '1' || eKey == '3'))
        {
          serialPrintLine("Holding button");
          buttonHoldTime = millis();
          int intKey = eKey - '0';
          buttonPressed = intKey;
        }
        break;

    case RELEASED:
      serialPrintLine("Released button");
      buttonHoldTime = 0;
      buttonPressed = -1;
      break;
   }
}

void setup()
{
  if(debugThroughComp)
  {
    Serial.begin(115200);
    while (!Serial) {
      ; // wait for serial port to connect
    }
  }

  ss.begin(9600);
  ss.setTimeout(500);

  keypad.addEventListener(keypadEvent);   
  pinMode(SIGNAL_RED_PIN, OUTPUT);  
  pinMode(SIGNAL_GRN_PIN, OUTPUT);
  pinMode(SIGNAL_YLW_PIN, OUTPUT);
  
  digitalWrite(SIGNAL_RED_PIN, LOW); 
  digitalWrite(SIGNAL_GRN_PIN, HIGH);
  digitalWrite(SIGNAL_YLW_PIN, HIGH);

  pinMode(DISPLAY_CS, OUTPUT);
  pinMode(DISPLAY_SDI, OUTPUT);
  pinMode(DISPLAY_SCL, OUTPUT);
  display_init();

  pinMode(SWITCH_PIN_OUT, OUTPUT);
  pinMode(SWITCH_PIN_IN, INPUT);

  setDisplay("OBrien Radio", "     Welcome Colin!", 0);

  delay(1000);

  int switchNormalInt = EEPROM.read(12);

  if (switchNormalInt < 0 || switchNormalInt > 1 || isnan(switchNormalInt))
  {
    EEPROM.write(12, true);
    switchNormal = true;
  }
  else
  {
    switchNormal = switchNormalInt;
  }
  
  if (switchNormal)
  {
    //switch the switch to NORMAL
    analogWrite(SWITCH_PIN_OUT, 255);
  }
  else
  {
    //switch the switch to REVERSE
    analogWrite(SWITCH_PIN_OUT, 0);
  }
  
  setDisplay("IAIS Radio", "     STARTING UP", 0);

  bool foundBoard = reset();
  serialPrintLine("Found Board: " + String(foundBoard));

  //get saved variables
  volume = EEPROM.read(0);
  if (volume < 0 || volume > 100 || isnan(volume))
  {
     volume = 100;
     EEPROM.write(0, volume);
     Serial.println("Set Volume to 100");
  }
  else
  {
    volDown();
    volDown();
    volDown();
    volDown();
    setVolume(volume);
  }

  delay(15);
  
  setDisplay("Volume: " + String(volume), "", 0);
  
  delay(2000);
  float f = 0.00f;
  EEPROM.get(6, f);
  serialPrintLine( "Read float from EEPROM: " );
  if(debugThroughComp)
  {
    Serial.println( f, 2 );  //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
  }
  
  if (f < 10 || f > 999 || isnan(f))
  {
     timeout = 600;
     EEPROM.put(6, 600.00);
     serialPrintLine("Set Timeout to 600");
  }
  else
  {
     timeout = (int)f;
     serialPrintLine("Read Timeout");
  }

  setDisplay("Timeout: " + String(timeout), "", 0);

  delayTime = timeout * 1000;
  delay(2000);
  setDisplay("IAIS Radio", "", iaisRadioOffset);
}
 
void loop()
{
  if (delayRunning && ((millis() - delayStart) >= delayTime) && millis() > delayTime) 
  {
    delayRunning = false; // finished delay -- single shot, once only
    setSignalStatus(0, false);
    if(debugThroughComp)
    {
      Serial.println(millis());
      Serial.println("Reset signal status");
    }
  }

  if (currentInput != "" && ((millis() - delayResetScreenDelay) >= delayResetScreenTime) && millis() > delayResetScreenDelay)
  {
    if(debugThroughComp)
    {
      Serial.println(millis());
      Serial.println(delayStart);
    }
    currentInput = "";
    setDisplay("IAIS Radio", "", iaisRadioOffset);
  }

  if (buttonPressed > -1 && currentInput == "" && buttonHoldTime > 0 && millis() > (buttonHoldTime + buttonHoldDelay))
  {
    if (buttonPressed == 1)
    {
       //set switch normal
       serialPrintLine("Switch is Normal");
       //set the display to showing the movement
       setDisplay("IAIS Radio", "Switch Normal", iaisRadioOffset);
       
       currentInput = "";
       //set the red LED to on and the rest off
       setSignalStatus(0, true);
       //move the switch machine
       bool moved = moveSwitch(true);

       if(!moved)
       {
          setDisplay("IAIS Radio", "SWITCH ERROR", iaisRadioOffset);
          delay(5000);
       }
       
       //set the green LED to on and the rest off
       setSignalStatus(2, true);

       // start delay
       delayTime = timeout * 1000;
       delayStart = millis();
       delayRunning = true;
       
       //Play the detector response sound - Switch is normal
       playDTMF('N');
       
       //delay the input period for a few seconds
       delay(10000);
       
       setDisplay("IAIS Radio", "", iaisRadioOffset);
    }
    else if (buttonPressed == 3)
    {
       //set the display to showing the movement
       setDisplay("IAIS Radio", "Switch Reverse", iaisRadioOffset);
       currentInput = "";
       //set the red LED to on and the rest off
       setSignalStatus(0, true);
       //move the switch machine
       bool moved = moveSwitch(false);
  
       if(!moved)
       {
          setDisplay("IAIS Radio", "SWITCH ERROR", iaisRadioOffset);
          delay(5000);
       }
       
       //set the yellow LED to on and the rest off
       setSignalStatus(1, true);
       
       // start delay
       delayTime = (long)timeout * 1000;
       delayStart = millis();
       delayRunning = true;
       
       //Play the detector response sound - Switch is reverse
       playDTMF('R');
       
       //delay the input period for a few seconds
       delay(10000);
       
       setDisplay("IAIS Radio", "", iaisRadioOffset);
    }

    buttonPressed = -1;
    buttonHoldTime = 0;
  }
  
  keypad.getKey();
}