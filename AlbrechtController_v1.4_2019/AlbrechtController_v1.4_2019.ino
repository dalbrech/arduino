#define version_name "AlbrechtController_v1.42_2019" // 2019.06.18 DRA

/** ------------------------------
Control valves at specific frames during a streaming acquisition
Input trigger from camera comes from pin 2
Output trigger mirrors input for controlling the illumination on pin 7
Valves are triggered to open and close at specific frames according to the serial command:
LED1 and LED8 are on pins 5, 6
Brightfield LED on Pin 8

//NEW SYNTAX
   ***new format: A/a = valve1 on/off, L/l = LED1 on/off (full power)
   A50,a100,A150,a200,L250,l300
**/

/***************************************************
  Uses touchscreen Adafruit ILI9341 captouch shield
  ----> http://www.adafruit.com/products/1947

  Check out the links above for our tutorials and wiring diagrams

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
****************************************************/


#include <Adafruit_GFX.h>       // Core graphics library
#include <SPI.h>                // this is needed for display
#include <Adafruit_ILI9341.h>   // TFT display
#include <Wire.h>               // this is needed for FT6206
#include <Adafruit_FT6206.h>    // Touchscreen
#include <Arduino.h>

// The FT6206 uses hardware I2C (SCL/SDA) ** these are Pins A4,A5 - DO NOT USE THEM!
Adafruit_FT6206 ctp = Adafruit_FT6206();

// The display also uses hardware SPI, plus #9 & #10
#define TFT_CS 10
#define TFT_DC 9
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

#define TRIGGERIN  2  // for interrupt 0 (CAM IN)
#define TRIGGEROUT_FL  7  // Digital out for fluorescence excitation (FL OUT - Lumencor)
#define TRIGGEROUT_BF  8  // Digital out for brightfield (BF OUT)

#define LED1 5    //  PWM for adjustable LED intensity (or stimulus voltage)
#define LED2 6    //  PWM for adjustable LED intensity

#define VALVE1 14    // TTL for valves, high = open  
#define VALVE2 15    // A0 = 14, A1 = 15, A2 = 16, etc 
#define VALVE3 16

#define GRAY 0x7BEF

// Variable sizes are limited to serial commands of 128 char, and up to 32 switches
// Expand as needed if patterns appear truncated!
#define INPUT_SIZE 128 // max input serial string size
#define MAX_SWITCHES 32 // max number of switch frames

// Following applies only to BuckPuck LEDs. Other sources may use TTL high = on, low = off
//const int BF_LEDON = 210; // Intensity control for BF; <80 = max; >220 = off; ~linear in between, 150 = 50% current
//const int BF_OFF = 255;

#define xdiv 2            // display x-scaling (200 pts display, so 1 = 200 fr, 2 = 400 fr, etc)

byte FL_ON = LOW;         // should be HIGH for Mightex; LOW for Lumencor
byte BF_ON = HIGH;        // should be HIGH for Zeiss, LOW for buckpuck
byte INPUT_ON = LOW;      // depends on and should match Micromanager settings (low for negative polarity)

byte FL_OFF = !FL_ON;
byte BF_OFF = !BF_ON;
byte INPUT_OFF = !INPUT_ON;

const int brightfieldInterval = 2;    // how often to pulse brightfield illumination, e.g. 2 = every other frame is BF; 3 = every 3rd frame...
byte testHz = 10;                     // test signal frequency
long int BFmicros = 0;                // brightfield illumination pulse duration (us). If 0, then follows input pulse
long int FLmicros = 0;                // fluorescent illumination pulse duration (us). If 0, then follows input pulse

// Initialize variables
int valve1state = LOW;         // default valve 1 OFF
int valve2state = HIGH;        // default valve 2 ON
int valve3state = LOW;         // default valve 3 OFF
int LED1out = 0;               // default value for LED1
int LEDintensity = 255;        // default LED intensity value
// add LED2out?

volatile long pulseCount = 0;
int valveSwitchCount = 0;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean getTouchPoint = false;   // has the screen been touched
boolean updateDisplay = true;    
boolean manualEntry = false;
boolean switchToNextValve = false;   // is there a state change to be made
boolean setLEDintensity = false;     // is there an intensity level change
boolean debug = true;

boolean FL = true;            // default power-on settings: FL only
boolean BF = false;
boolean testMode = false;
boolean editSettings = false;

// Initialize variables
int Frames[MAX_SWITCHES];       // vector with frame numbers
int LED1levels[MAX_SWITCHES];   // LED1 state at each frame number
int V1states[MAX_SWITCHES];     // Valve 1 state at each frame number
int V2states[MAX_SWITCHES];     // ...   2
int V3states[MAX_SWITCHES];     // ...   3

int xos = 5;       // offset pixels for display
int yos = 255;

int prevx = 0;     // prior touch point coordinate
int prevy = 0;
long int ms = millis();   // elapsed time in ms
long int testms = ms;
float fps;                // frame rate 

char input[INPUT_SIZE + 1];

const char keypad[] = "123456789i0,";

void setup()
{
    for (int i=0; i<MAX_SWITCHES; i++) {    // initialize all vectors as -1
        Frames[i] = -1;
        LED1levels[i] = -1;
    }
    
    // Set Pin settings
    pinMode(TRIGGERIN, INPUT_PULLUP);
    pinMode(TRIGGEROUT_FL, OUTPUT);
    pinMode(TRIGGEROUT_BF, OUTPUT);
    pinMode(VALVE1, OUTPUT);
    pinMode(VALVE2, OUTPUT);
    pinMode(VALVE3, OUTPUT);
    pinMode(LED1, OUTPUT);
    // Add LED2?

    // Initialize outputs
    digitalWrite(TRIGGEROUT_FL, FL_OFF);
    //if (!BF) analogWrite(LED2, BF_OFF);
    digitalWrite(TRIGGEROUT_BF, BF_OFF);
    digitalWrite(VALVE1, valve1state);
    digitalWrite(VALVE2, valve2state);
    digitalWrite(VALVE3, valve3state);

    digitalWrite(LED1, LOW);
    // Add LED2?

    // Define Interrupts. 0 (pin 2 detects change at CAM IN)
    //                    1 (pin 3 detects screen touch)
    attachInterrupt(0, triggerChange, CHANGE);
    attachInterrupt(1, screenTouched, LOW);
    
    // start serial port
    Serial.begin(115200);
    
    Serial.println(F("Capacitive touchscreen started"));
    Serial.println();
    Serial.println(F("------------------------------"));
    Serial.println(F("Valve Commands:"));
    Serial.println(F("  A###, a###    valve 1 on, off at frame ###"));
    Serial.println(F("  B###, b###    valve 2 on, off ..."));
    Serial.println(F("  C###, c###    valve 3 ..."));
    Serial.println(F("Stimulus (LED) Commands:"));
    Serial.println(F("  L###, l###    LED1 on, off at frame ###"));
    Serial.println(F("  i###          sets intensity of next LED [0=off, 255=max]"));
    Serial.println();
    Serial.println(F("example:  A50,a100,C100,c125,i100,L200,i200,L225,l250"));
    Serial.println();
    Serial.println(F("------------------------------"));
    Serial.println(F("Manual Commands:"));
    Serial.println(F("  v1on, v1off   immediate valve 1 on, off"));
    Serial.println(F("  v2on, v2off   immediate valve 2 on, off"));
    Serial.println(F("  v3on, v3off   immediate valve 3 ..."));
    Serial.println();
    Serial.println(F("  =N, =P      set Fluor. polarity to Negative or Positive"));
    Serial.println(F("  =n, =p      set CAM IN polarity to Negative or Positive"));
    Serial.println(F("              can combine: =Nn  set both to negative [default]"));
    Serial.println();
    Serial.println(F("  _F, _B      set Fluor. or brightfield illumination"));
    Serial.println(F("              can combine: _BF set both to alternate "));
    Serial.println();
    Serial.println(F("  ~f###       set Fluor. pulse to ### microseconds (us)"));
    Serial.println(F("  ~b###       set brightfield pulse to ### microseconds (us)"));
    Serial.println(F("              if set to 0, follows the CAM IN trigger [default]"));
    Serial.println();
    Serial.println(F("  T           start test signal"));
    Serial.println(F("  X           end test signal"));
    Serial.println();
    Serial.println(F("  reset       reset Arduino"));
    Serial.println();
    Serial.println(F("------------------------------"));
    Serial.println(F(version_name)); 
      
    // initialize LCD display
    tft.begin();
    if (! ctp.begin(40)) {  // pass in 'sensitivity' coefficient
      Serial.println(F("Couldn't start FT6206 touchscreen controller"));
    //  while (1);
    }

    // Display LCD screen
    resetScreen();   
    updateDisplay = true; 
}

// Define the software reset function
void(* resetFunc) (void) = 0; //declare reset function @ address 0




void loop()
{
    // digitalWrite(VALVE3, LOW);

    // Check for serial command to define and start the pulse counting and valve control.
    int i = 0;
    //int j = 0;

    while (Serial.available() > 0 | manualEntry)
    {

        if (Serial.available()) {
            // Get next command from Serial (add 1 for termination character)
            byte size = Serial.readBytes(input, INPUT_SIZE);
            // Add the final 0 to end the string
            input[size] = 0;
            inputString = String(input);
        } else resetScreen();
        manualEntry = false;
                
        Serial.print(F("Input: [")); Serial.print(inputString); Serial.print("] ");
        
        boolean timingEntry = true; // assume timing pattern until evidence otherwise

///////////////////// PARSE MANUAL COMMANDS HERE //////////////////////
        
        // Check for manual valve command: v1on/v1off/v2on/v2off, etc
        char* ch = strchr(input, 'v');
        if (ch != 0)
        {
            ++ch;
            int valvenum = atoi(ch); // get valve number 
            ch += 2;                 // move char pointer

            if (valvenum == 1)
            {
                valve1state = (strchr(input, 'on') != 0);
                digitalWrite(VALVE1, valve1state);
                Serial.print(F("Valve 1 switch to: ")); Serial.println(valve1state);
            }
            if (valvenum == 2)
            {
                valve2state = (strchr(input, 'on') != 0);
                digitalWrite(VALVE2, valve2state);
                Serial.print(F("Valve 2 switch to: ")); Serial.println(valve2state);
            }
            if (valvenum == 3)
            {
                valve3state = (strchr(input, 'on') != 0);
                digitalWrite(VALVE3, valve3state);
                Serial.print(F("Valve 3 switch to: ")); Serial.println(valve3state);
            }
            timingEntry = false;
            updateDisplay = true;
        }

        // Check for selection of brightfield vs fluorescence illumination
        ch = strchr(input, '_');  // _F = fluor, _B = brightfield, _BF or _FB = both
        if (ch != 0)
        {
            ++ch;

            BF = (strchr(input, 'B') != 0);
            FL = (strchr(input, 'F') != 0);
                        
            drawButton(3,5,"BF",ILI9341_GREEN, BF);
            drawButton(2,5,"FL",ILI9341_BLUE, FL);
                        
            Serial.print(F("Changing FL, BF setting to: ")); Serial.print(FL); Serial.print(", "); Serial.println(BF);
            timingEntry = false;
        }

        // Check for selection of OUTPUT and CAM_IN polarity (OUTPUT = capital, CAM_IN = lower)
        ch = strchr(input, '=');  // =Nn = negative FL, negative CAM IN,
                                  // =Np = negative FL, positive CAM IN, etc
        if (ch != 0)
        {
            ++ch;
            INPUT_ON = LOW;
            INPUT_OFF = LOW;
            if (strchr(input, 'P') != 0) {
                FL_ON = HIGH;
            }
            if (strchr(input, 'p') != 0) {
                INPUT_ON = HIGH; 
            }
            FL_OFF = !FL_ON;
            INPUT_OFF = !INPUT_ON;
            Serial.print(F("FL_out, CAM_in Polarities: ")); Serial.print(FL_ON); Serial.print(", "); Serial.println(INPUT_ON);
            timingEntry = false;
        }

        // Check for manual definition of pulse timing use ~f## for FL and ~b## for BF
        ch = strchr(input, '~');  // e.g. ~f10 = 10 us, ~f10000 = 10 ms
        if (ch != 0)
        {
            ++ch;
            
            if (strchr(input, 'f') != 0) {
                ++ch;
                FLmicros = atoi(ch);
                Serial.print(F("Changing FL pulse duration to: "));
                Serial.print(FLmicros);
                Serial.println(" us");
                String label = "FL| |";
                if (FLmicros > 0) {
                    if (FLmicros < 1000) {
                        label += FLmicros; label += "us";
                    } else {
                        label += FLmicros/1000; label += "ms";
                    }
                } else label += "pulse";
                drawButton(2,5,label,ILI9341_BLUE, FL);

            } else if (strchr(input, 'b') != 0) {
                ++ch;
                BFmicros = atoi(ch);
                Serial.print(F("Changing BF pulse duration to: "));
                Serial.print(BFmicros);
                Serial.println(" us");
                String label = "BF| |";
                if (BFmicros > 0) {
                    if (BFmicros < 1000) {
                        label += BFmicros; label += "us";
                    } else {
                        label += BFmicros/1000; label += "ms";
                    }
                } else label += "pulse";
                drawButton(3,5,label,ILI9341_GREEN, BF);

            }         
            timingEntry = false;
        }
                
        // Check to start test mode or stop test mode 
        // (internally-generated camera signals)
        ch = strchr(input, 'T');  // test mode start
        if (ch != 0)
        {
            ++ch;
            testMode = true;
            setTestMode(testMode);
            timingEntry = false;
            Serial.flush();
        }
        ch = strchr(input, 'X');  // test mode end
        if (ch != 0)
        {
            ++ch;
            testMode = false;
            setTestMode(testMode);
            timingEntry = false;
            Serial.flush();
        }
        
        ch = strchr(input, 'reset');  // hard reset  
        if (ch != 0) resetFunc();     //call reset

///////////////////////////  PARSE TIMING COMMANDS HERE ///////////////////
                        
        if (timingEntry)
        {
            V1states[0] = 0;
            V2states[0] = -1;    // important: -1 means don't change it!
            V3states[0] = -1;    // ...
            LED1levels[0] = 0;
            Frames[0] = 0;
            setLEDintensity = false;
            i = 1;

            Serial.print(F("Parse: "));
            // Parse timing and LED intensity commands
            char* command = strtok(input, " ,");
            while (command != 0)
            {
                //Serial.print(" "); Serial.println(command);

                // Initialize next command as same as prior step
                //  (so we don't need to do it repeatedly, and
                //  they will be reset if no valid command is found)
                LED1levels[i] = LED1levels[i-1];
                V1states[i] = V1states[i-1];
                V2states[i] = V2states[i-1];
                V3states[i] = V3states[i-1];
                
                char firstChar = *command;
                if (debug) { Serial.print("\n ["); Serial.print(firstChar); Serial.print(']');}
                switch (firstChar) {
                    case 'i':         // to set current intensity
                        setLEDintensity = true;
                        break;
                    case 'A':         // valve 1 open
                        V1states[i] = 1; 
                        break;
                    case 'a':         // valve 1 closed
                        V1states[i] = 0; 
                        break;
                    case 'B':         // valve 2 open
                        V2states[i] = 1; 
                        break;
                    case 'b':         // valve 2 closed
                        V2states[i] = 0; 
                        break;
                    case 'C':         // valve 3 open
                        V3states[i] = 1; 
                        break;
                    case 'c':         // valve 3 closed
                        V3states[i] = 0; 
                        break;
                    case 'L':
                        LED1levels[i] = LEDintensity;
                        break;
                    case 'l':
                        LED1levels[i] = 0;
                        break;
                    default:
                        LED1levels[i] = -1;
                        V1states[i] = -1;
                        V2states[i] = -1;
                        V3states[i] = -1;
                    break;
                }
                *command = 0;
                ++command;
                if (setLEDintensity)
                {
                    LEDintensity = atoi(command);
                    if(debug) { Serial.print(F(" LED intensity set to: ")); Serial.print(LEDintensity); }
                    setLEDintensity = false;
                }
                else 
                {
                    Frames[i] = atoi(command);
                    if(debug) { Serial.print(F(" at frame ")); Serial.print(Frames[i]); }
                    i++;
                }

                // Find the next command in input string
                command = strtok(0, " ,");
            }

            Serial.print(F("\n# Frame switches: ")); Serial.print(i);
            //Serial.print(F(", LED1levels ")); Serial.println(j);
            Serial.println();

            /*
            // Sort output by time in ascending order
            int idx[i]; 
            for (int k = 0; k < i; k++) idx[k] = k; // assume initially in ascending order
            for (int k = 0; k < i; k++) {           // then test and swap as necessary
                for (int k2 = k; k2 < i; k2++) {    // step through each additional value
                    if (Frames[k2] < Frames[k]) { 
                      int temp = Frames[k]; Frames[k]=Frames[k2]; Frames[k2]=temp; 
                          temp = idx[k];    idx[k]=idx[k2];       idx[k2]=temp; 
                          temp = LED1levels[k]; LED1levels[k]=LED1levels[k2]; LED1levels[k2]=temp;
                          temp = V1states[k]; V1states[k]=V1states[k2]; V1states[k2]=temp;
                          temp = V2states[k]; V2states[k]=V2states[k2]; V2states[k2]=temp;
                          temp = V3states[k]; V3states[k]=V3states[k2]; V3states[k2]=temp;
                    }
                }
                Serial.print(idx[k]); Serial.print(", ");
            }
            */

            for (int z = i; z < MAX_SWITCHES; z++) {
              Frames[z] = -1;      // blank out any previous frame switch settings
              LED1levels[z] = -1;  // blank out any previous intensity settings
              V1states[z] = -1;    // blank out any previous valve settings
              V2states[z] = -1;    // ...
              V3states[z] = -1;    // ...
            }

            // display vectors info for debugging
            Serial.println("Fr\t v1\t v2\t v3\t LED1");

            // check for non-controlled valves and out-of-order values
            int v1used, v2used, v3used;          
            for (int k = 0; k < MAX_SWITCHES; k++) {
              if (k>0 && k<i && Frames[k]<Frames[k-1]) {
                  Serial.println(F("\n*** WARNING: Switch times out of order. May not switch properly! ***"));
                  tft.setTextColor(ILI9341_BLACK, ILI9341_RED);
                  tft.setTextSize(1);
                  tft.setCursor(5,20);
                  tft.print(F(" WARNING: Switch times out of order "));
              }
              v1used += (V1states[k]>= 0);
              v2used += (V2states[k]>= 0);
              v3used += (V3states[k]>= 0);
                            
              if (debug) { 
                Serial.print(Frames[k]); Serial.print('\t'); 
                Serial.print(V1states[k]); Serial.print('\t'); 
                Serial.print(V2states[k]); Serial.print('\t'); 
                Serial.print(V3states[k]); Serial.print('\t'); 
                Serial.print(LED1levels[k]); Serial.println('\t'); 
                //if (V2states < 0) Serial.print("No V2states");
              }
            }
            Serial.print("\t"); Serial.print(v1used);
            Serial.print("\t"); Serial.print(v2used);
            Serial.print("\t"); Serial.println(v3used);
                   
            if (i > 0)                               // if there is timing data, then:
            {
                pulseCount = -2;                     // Initialize pulse count
                valveSwitchCount = 0;                // Initialize valve switch count
                digitalWrite(VALVE1, LOW);           // Initialize valve 1 off
                digitalWrite(VALVE2, HIGH);          // Initialize valve 2 ON
                //digitalWrite(VALVE3, LOW);         // Initialize valve 3 off
                //digitalWrite(LED1, LOW);           // Initialize LED1 off
                            
                tft.fillRect(1,45,tft.width(),4,ILI9341_BLACK);
                tft.fillRect(xos,yos,64,48,ILI9341_BLACK);
                if (v1used) displayGraph(30,65,"V1",Frames,V1states,1,12.0);
                if (v2used) displayGraph(30,85,"V2",Frames,V2states,1,12.0);
                if (v3used) displayGraph(30,105,"V3",Frames,V3states,1,12.0);
                displayGraph(30,142,"LED1",Frames,LED1levels,255,0.1);
                
                //if (~BF) analogWrite(LED2, BF_OFF);
                drawButton(3,5,"BF",ILI9341_GREEN, BF);
                drawButton(2,5,"FL",ILI9341_BLUE, FL);

            }
            /*
            else if (j == 1) {
                LED1out = LED1levels[0];
                Serial.println(LED1out);
                if (LED1out >= 0) {
                    analogWrite(LED1, LED1out);
                    Serial.println(LED1out);
                }
            }
            */
            Serial.read();   //
        }
    }

    if (testMode) {    // establish a 1ms pulse every 100ms test input
        //long delayms = testms + 60000 - millis();
        int delayms = testms + (1000/testHz) - millis();
        if (delayms > 0) delay(delayms);
        
        digitalWrite(TRIGGERIN, INPUT_ON);
        delayMicroseconds(1000);
        digitalWrite(TRIGGERIN, INPUT_OFF);
        
        //Serial.print('.');
        Serial.println(testms);
    }

    // Update valves if pulseCount equals the next switch frame
    if (switchToNextValve)
    {
        while (Frames[valveSwitchCount] > 0 && pulseCount >= Frames[valveSwitchCount]) 
        {
            LED1out = LED1levels[valveSwitchCount];
            if (LED1out >= 0) analogWrite(LED1, LED1out);

            // Get desired valve states and set. Ignore if value is -1
            valve1state = V1states[valveSwitchCount];
            if (valve1state >= 0) digitalWrite(VALVE1, valve1state);
            valve2state = V2states[valveSwitchCount];
            if (valve2state >= 0) digitalWrite(VALVE2, valve2state);
            valve3state = V3states[valveSwitchCount];
            if (valve3state >= 0) digitalWrite(VALVE3, valve3state);
        
            valveSwitchCount++;
        }
        switchToNextValve = false;
    }

    if (updateDisplay) {
        //tft.fillRect(xos,yos,64,48,ILI9341_BLACK);
        tft.setTextSize(1);

        if (pulseCount <= 0) {
            tft.setTextColor(GRAY, ILI9341_BLACK);
            tft.setCursor(xos+ 0 * 6, yos+ 0 * 8);
            tft.print("frame next");
            tft.setCursor(xos+ 6 * 6, yos+ 3 * 8);
            tft.print("LED");
            tft.setCursor(xos+ 0 * 6, yos+ 3 * 8);
            tft.print("v1:");
            tft.setCursor(xos+ 0 * 6, yos+ 4 * 8);
            tft.print("v2:");
            tft.setCursor(xos+ 0 * 6, yos+ 5 * 8);
            tft.print("v3:");
        }
        
        tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
        tft.setCursor(xos+ 0 * 6, yos+ 1 * 8);
        tft.print(pulseCount); tft.print(" ");
        tft.setCursor(xos+ 6 * 6, yos+ 1 * 8);
        tft.print(Frames[valveSwitchCount]); tft.print("  ");
        tft.setCursor(xos+ 3 * 6, yos+ 3 * 8);
        tft.print(valve1state); tft.print(" ");
        tft.setCursor(xos+ 3 * 6, yos+ 4 * 8);
        tft.print(valve2state); tft.print(" ");
        tft.setCursor(xos+ 3 * 6, yos+ 5 * 8);
        tft.print(valve3state); tft.print(" ");
        tft.setCursor(xos+ 6 * 6, yos+ 4 * 8);
        tft.print(LED1out); tft.print("  ");
        if (pulseCount % (int) fps == 0) {            // display fps every ~1s
            tft.setCursor(xos+ 0 * 6, yos+ 6 * 8);
            tft.print(fps); tft.print(" Hz ");
        }
          //tft.drawLine( 30 + pulseCount/2 - 1, 65-10-5, 30 + pulseCount/2 - 1, 65-10-10, ILI9341_BLACK);
        if (pulseCount % xdiv == 1) tft.drawLine( 30 + pulseCount/xdiv, 46, 30 + pulseCount/xdiv, 43, ILI9341_GREEN);
            updateDisplay = false;
    }

    if (getTouchPoint) {

        TS_Point p = ctp.getPoint();
        int x = map(p.x,0,240,240,0);
        int y = map(p.y,0,320,320,0);
        if ( (p.x * p.y > 0) && (abs(prevx-x)>5) && (abs(prevy-y)>5) ) 
        {
            Serial.print(x); Serial.print(","); Serial.print(y);
            byte bx = byte(x/48);
            byte by = byte(y/48);
        
            if (bx == 2 && by == 5) {
                FL = !FL;
                String label = "FL| |";
                if (FLmicros > 0) {
                  if (FLmicros < 1000) {
                    label += FLmicros; label += "us";
                  } else {
                    label += FLmicros/1000; label += "ms";
                  }
                } else label += "pulse";
                drawButton(2,5,label,ILI9341_BLUE, FL);
            }
            
            if (bx == 3 && by >= 5) {
                if (by > 5) {
                    BFmicros *= 2;
                    if (BFmicros > 1000) BFmicros = 1;
                } else BF = !BF;
                String label = "BF| |"; label += BFmicros; label += "us";
                drawButton(3,5,label,ILI9341_GREEN, BF);
            }
                
            if (bx == 4 && by >= 5) {
                if (by > 5) {
                    testHz *= 2;
                    if (testHz > 100) testHz = 1;  // double test frequency from 10 - 100, then reset to 1 fps
                } else testMode = !testMode;
                setTestMode(testMode);
            }
            
            if (bx == 4 && by == 4) {
                Serial.print("Reset");
                //Serial.println(inputString);
                manualEntry = true;
                inputString.toCharArray(input,INPUT_SIZE+1);
            }
            
            if (bx == 3 && by == 4) {
                Serial.print("Preset");
                manualEntry = true;
                inputString = "A50,L100,a150,l200\n";
                inputString.toCharArray(input,INPUT_SIZE+1);
            }
            
            if (bx == 2 && by == 4) {
                Serial.print("Enter program");
                //getSettings();
                manualEntry = true;
                inputString = getSettings(inputString,"Enter new command:");
                inputString.toCharArray(input,INPUT_SIZE+1);
            }
            
            if (bx == 1 && by == 4) {
                Serial.print("Settings");
                editSettings = !editSettings;
                drawButton(1,4,"Set-|tings", GRAY, editSettings);

                if (editSettings)
                {
                    tft.fillRect(1,45,tft.width(),100,ILI9341_BLACK);
                    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
                    tft.setCursor(64,50); tft.print(F("Signal polarity:"));
                    tft.setCursor(52,80); tft.print(F("Camera"));
                    tft.setCursor(180,80); tft.print(F("Fluor."));
                    drawButton(1,2,(INPUT_ON ? "Pos| | +" : "Neg| | -"),ILI9341_WHITE,INPUT_ON);
                    drawButton(3,2,(FL_ON ? "Pos| | +" : "Neg| | -"),ILI9341_BLUE,FL_ON);
                } else {
                    manualEntry = true;
                    inputString.toCharArray(input,INPUT_SIZE+1);
                }
            }
            if (bx == 1 && by == 2 & editSettings) {
                INPUT_ON = !INPUT_ON;
                INPUT_OFF = !INPUT_OFF;
                drawButton(1,2,(INPUT_ON ? "Pos| | +" : "Neg| | -"),ILI9341_WHITE,INPUT_ON);
            }
            
            if (bx == 3 && by == 2 & editSettings) {
                FL_ON = !FL_ON;
                FL_OFF = !FL_OFF;
                drawButton(3,2,(FL_ON ? "Pos| | +" : "Neg| | -"),ILI9341_BLUE,FL_ON);
            }
            Serial.print(" --> "); Serial.print(bx); Serial.print(","); Serial.println(by);
        }
        
        prevx = x; prevy = y;
        getTouchPoint = false;
    }
}

void setTestMode( boolean testMode)
{
    String label = "test| |"; label += testHz; label += "Hz";
    drawButton(4,5,label,ILI9341_RED, testMode);
    if (testMode) {
        pinMode(TRIGGERIN, OUTPUT);
    } else {
        pinMode(TRIGGERIN, INPUT);
    }
}

void drawButton( int xpos, int ypos, String buttonName, unsigned int buttonColor, boolean pressed)
{
    if (pressed) {
        tft.fillRoundRect(48*xpos+1,48*ypos+1,48-2,48-2,10,buttonColor);
        tft.setTextColor(ILI9341_BLACK, buttonColor);
    } else {
        tft.fillRoundRect(48*xpos+1,48*ypos+1,48-2,48-2,10,ILI9341_BLACK);
        tft.drawRoundRect(48*xpos+1,48*ypos+1,48-2,48-2,10,buttonColor);
        tft.setTextColor(buttonColor, ILI9341_BLACK);
    }

    int buttonLine = 0;
    char buttonChars[INPUT_SIZE + 1];
    buttonName.toCharArray(buttonChars, INPUT_SIZE+1);
    char* buttonText = strtok(buttonChars, "|");
        while (buttonText != 0)
        {
            tft.setCursor(48*xpos+10,48*ypos+12+buttonLine*8);
            tft.print(buttonText);
            buttonText = strtok(0, "|");
            buttonLine++;
        }
}

void displayGraph( int xos, int yos, String graphName, int xlist[], int ylist[], int maxy, float ysc )
{
    tft.fillRect(0,yos-maxy*ysc,tft.width(),ysc*maxy+20,ILI9341_BLACK); // clear the row
        //tft.setCursor(xos-25, yos-25);
    tft.setCursor(xos-25, yos-maxy*ysc);
    tft.setTextColor(ILI9341_RED);
    tft.setTextSize(1);
    tft.print(graphName);
        
    int i = 0;
    int x1 = -1;
    int y1 = -1;
    int x2,y2;
    int color;
    for (i=0; (i<MAX_SWITCHES-1) & (xlist[i] > -1); i++) {
          if (i>0) x1 = xlist[i-1];
          x2 = xlist[i];
          if (i>0) y1 = ylist[i-1];
          y2 = ylist[i];
          //Serial.print(x1); Serial.print(","); Serial.print(y1); Serial.print(","); Serial.print(x2); Serial.print(","); Serial.println(y2);
          if (y1>=0) color = ILI9341_WHITE;
          else {
            y1 = 0; y2 = max(0,y2); // put a blue mark to indicate no value
            color = ILI9341_BLUE;
          }
          tft.drawLine( xos+x1/xdiv, yos-y1*ysc, xos+x2/xdiv, yos-y1*ysc, color);
          tft.drawLine( xos+x2/xdiv, yos-y1*ysc, xos+x2/xdiv, yos-y2*ysc, color);
          //Serial.println(i);
      }
      //Serial.print(i); Serial.println(x2);
      tft.drawLine( xos+x2/xdiv, yos-y2*ysc, xos+200, yos-y2*ysc, ILI9341_WHITE);
      for (int i=0; i<=200; i+=50/xdiv) {
      tft.drawPixel( xos + i, yos-maxy*ysc-4, ILI9341_RED);
      tft.drawPixel( xos + i, yos+4, ILI9341_RED);
  }
}

String getSettings(String inputString, String prompt)
//void getSettings()

{
/*
    tft.fillRect(0,20,tft.width(),280,ILI9341_BLACK);

    drawButton(1,2,"  1",ILI9341_WHITE, false);
    drawButton(2,2,"  2",ILI9341_WHITE, false);
    drawButton(3,2,"  3",ILI9341_WHITE, false);
    drawButton(1,3,"  4",ILI9341_WHITE, false);
    drawButton(2,3,"  5",ILI9341_WHITE, false);
    drawButton(3,3,"  6",ILI9341_WHITE, false);
    drawButton(1,4,"  7",ILI9341_WHITE, false);
    drawButton(2,4,"  8",ILI9341_WHITE, false);
    drawButton(3,4,"  9",ILI9341_WHITE, false);
    drawButton(1,5,"  i",ILI9341_WHITE, false);
    drawButton(2,5,"  0",ILI9341_WHITE, false);
    drawButton(3,5,"  ,",ILI9341_WHITE, false);

    drawButton(0,5," Go|Back",ILI9341_RED, false);
    drawButton(4,5," OK!",ILI9341_GREEN, true);
        delay(1000);
        String newString = "";
    //noInterrupts();
    boolean entryDone = false;
    boolean getTouchPoint = false;
            tft.setTextColor(0x7BEF, ILI9341_BLACK);
    tft.setTextSize(1);
    tft.setCursor(20,30);
    tft.print(prompt);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setCursor(20,40);
        while (!entryDone) {
            if (ctp.touched()) {
            TS_Point p = ctp.getPoint();
            byte bx = byte(map(p.x,0,240,240,0)/48);
            byte by = byte(map(p.y,0,320,320,0)/48);
            if (bx == 0 && by == 5) {
                entryDone = true;
                return inputString;
                Serial.println("Cancel");
            }
            if (bx == 4 && by == 5) {
                entryDone = true;
                newString += "\n";
                return newString;
                Serial.println("OK");
            }
            if (bx >= 1 && bx <= 3 && by >= 2) {
                byte val = (by-2)*3+bx;
                newString += keypad[val-1];
                 tft.print(keypad[val-1]);
            }
            if (by < 2) {
                newString = newString.substring(0,newString.length()-1);
                tft.fillRect(0,40,tft.width(),48,ILI9341_BLACK);
                tft.setCursor(20,40);
                tft.print(newString);
            }
                }
        delay(150);
            }
    getTouchPoint = false;
    //manualEntry = true;
    //interrupts();
*/
}

void resetScreen()
{
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(GRAY, ILI9341_BLACK);
    tft.setTextSize(1);
    tft.setCursor(5,5);
    tft.print(F(version_name));

    drawButton(3,5,"BF",ILI9341_GREEN, BF);
    drawButton(2,5,"FL",ILI9341_BLUE, FL);
    drawButton(4,5,"test",ILI9341_RED, testMode);
    drawButton(1,4,"Set-|tings", GRAY, false);
    drawButton(2,4,"Enter|Prog.", GRAY, false);
    drawButton(3,4,"Pre-|set", GRAY, false);
    drawButton(4,4,"Reset", GRAY, false);

    tft.setCursor(xos+ 0 * 6, yos+ 0 * 8);
    tft.print("frame next");
    tft.setCursor(xos+ 6 * 6, yos+ 3 * 8);
    tft.print("LED");
    tft.setCursor(xos+ 0 * 6, yos+ 3 * 8);
    tft.print("v1:");
    tft.setCursor(xos+ 0 * 6, yos+ 4 * 8);
    tft.print("v2:");
    tft.setCursor(xos+ 0 * 6, yos+ 5 * 8);
    tft.print("v3:");
}

void triggerChange()
{
    if (digitalRead(TRIGGERIN) == INPUT_ON)  // input signal from camera to pulse the light
    {
        pulseCount++;
        Serial.print(pulseCount); Serial.print(' ');
        if (pulseCount == Frames[valveSwitchCount]) switchToNextValve = true;
        updateDisplay = true;
        fps = 1000.0 / (millis()-testms);
        testms = millis();

        // next pulse is fluorescent if only FL, OR both FL & BF and it's not BF's turn
        boolean fluor_pulse = (FL && (!BF || (BF && !(pulseCount % brightfieldInterval))));

        if (fluor_pulse) {
            digitalWrite(TRIGGEROUT_FL, FL_ON);
            Serial.print("/f");
            if (FLmicros > 0) {
                delayMicroseconds(FLmicros);          // if specified, define FL pulse
                digitalWrite(TRIGGEROUT_FL, FL_OFF);  // otherwise, follow TRIGGERIN 
                Serial.print("| ");
            }       
        } else {
            digitalWrite(TRIGGEROUT_BF, BF_ON);
            Serial.print("/b");
            if (BFmicros > 0) {
                delayMicroseconds(BFmicros);          // if specified, define BF pulse
                digitalWrite(TRIGGEROUT_BF, BF_OFF);  // otherwise, follow TRIGGERIN
                Serial.print("| ");
            }             
        }

        /*
        if (BF && FL) {
            if (pulseCount % brightfieldInterval) {
                analogWrite(LED2, BF_ON);
                delayMicroseconds(BFmicros);
                analogWrite(LED2, BF_OFF);
            } else {
                digitalWrite(TRIGGEROUT_FL, FL_ON);
            }
        } else {
            if (BF) {
                analogWrite(LED2, BF_ON); delayMicroseconds(BFmicros);
                analogWrite(LED2, BF_OFF);
            }
            if (FL) {
              digitalWrite(TRIGGEROUT_FL, FL_ON);
              if (FLmicros > 0) {
                delayMicroseconds(FLmicros);
                digitalWrite(TRIGGEROUT_FL, FL_OFF);
              }
            }
        }  */
    }
    else
    {
        digitalWrite(TRIGGEROUT_BF, BF_OFF);
        digitalWrite(TRIGGEROUT_FL, FL_OFF);
        Serial.print("\\ ");
    }
}

void screenTouched()
{
    if (millis()-ms > 100)
    {
        getTouchPoint = true;
        ms = millis();
    }
}

/*
int freeRam()
{
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
*/




