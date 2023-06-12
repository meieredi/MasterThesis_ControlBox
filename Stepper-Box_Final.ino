// Created by Eduard Meier, last updated: 09.05.2023

///////////////////////////////////////////////////////////////////
// REMARK /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

//----------------------------------------------------------------
// Additional files required (in the project's arduino folder) ---
//----------------------------------------------------------------

/*
1. Configuration files:   -   DEV_Config.cpp (C++)
                          -   DEV_Config.h (header)
2. Debugging files:       -   Debug.h (header)
3. Painting files:        -   GUI_Paint.cpp (C++) & GUI_Paint.h (header)
4. Image upload files:    -   ImageData.c (C)
                          -   ImageData.h (header)
5. OLED driver files:     -   OLED_Driver.cpp (C++)
                          -   OLED_Driver.h (header)
6. Font files:            -   font12.cpp (C++)
                          -   font12CN.cpp (C++)
                          -   font16.cpp (C++) 
                          -   font20.cpp (C++)
                          -   font20.cpp (C++)
                          -   font24.cpp (C++)
                          -   font24CN.cpp (C++)
                          -   font8.cpp (C++)
                          -   fonts.h (header)
*/

///////////////////////////////////////////////////////////////////
// Preliminaries //////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

//----------------------------------------------------------------
// Include the relevant libraries --------------------------------
//----------------------------------------------------------------

#include "OLED_Driver.h"    // Driver for the 1.5" rgb OLED
#include "GUI_paint.h"      // To displaly/paint content on OLED
#include "DEV_Config.h"     // Configuration file
#include "Debug.h"          // Debugging file
#include "ImageData.h"      // To display images on the OLED
//#include <Stepper.h>        // Driver for NEMA17 stepper motor

//----------------------------------------------------------------
// Define digital and analog pin occupation ----------------------
//----------------------------------------------------------------

// 1. Analog pins
#define acs712Pin A4            // Analog signal from current sensor 

// 2. Digital pins
#define upDownButtonPin 14      // Button for up/down navigation
#define rightLeftButtonPin 15   // Button for left/right navigation
#define returnButtonPin 16      // Button to return
#define enterButtonPin 17       // Button to enter

#define stepperDirPin 2         // Connected to stepper driver (DRV8825)
#define stepperStepPin 3
#define stepperEnPin 4

#define displayPin1 7           // Connected to 1.5" rgb OLED
#define displayPin2 8           // (4-pin SPI serial communication)
#define displayPin3 10
#define displayPin4 11
#define displayPin5 13

//----------------------------------------------------------------
// Initialize (and assign) constants and variables ---------------
//----------------------------------------------------------------

// Remark: initialized outside any function, all variables are global!

int i, j, k;                                      // Index variables
bool firstTime[] = {1, 1, 1, 1, 1};               // First time check for switch
bool buttonPressed;                               // Is button pressed? Yes -> true
bool enterPressed, returnPressed, upPressed, rightPressed; // Specific button pressed
bool buttonValues[] = {0, 0, 0, 0};               // Array to indicate pressed button
const int buttonElements = sizeof(buttonValues);  // Size of buttonValues array
int status = 0;                                   // Status in navigation menu
int timerStatus = 0;                              // Status of countdown timer
int oledNumRows = 128;                            // Number of pixel rows in OLED
int oledNumCols = 128;                            // Number of pixel columns in OLED
int rpm;                                          // Stepper rotation frequency [1/min]
int rpmMax = 500;                                 // Maximal rpm value that is allowed
int numbers[] = {0, 1, 2, 3, 4, 5, 6, 7, 8 , 9};  // Decimal numbers for rpm/time/shear selection
int digits[] = {1, 0, 0, 0, 0, 0, 0};             // Digits of a seven-digit number
String digitString;                               // String of selected digits
int digitDistance = 20;                           // Distance (horizontal) between displayed digits
int cursorDistance = 22;                          // Distance (vertical) between digit and cursor
int cursorRow = 60+cursorDistance;                // Cursor current row
int cursorCol = 0;                                // Cursor current column
bool timeMode;                                    // = 1 for time mode, = 0 for shear mode
int rpmRecommended[] = {100, 200, 300, 400};      // Recommended rpm values (well-characterized shear rates)
float shearRecommended[] = {437.48, 881.0, 1337.1, 1808.3}; // Simulated shear rates [1/s]
float shearRate;                                  // Shear rate corresponding to selected rpm [1/s]
int shearModeCols[] = {0, 82};                    // Cursor columns for rpm selection in shear mode
int shearModeRows[] = {60, 90};                   // Cursor rows for rpm selection in shear mode
int timeModeCols[] = {0, digitDistance, 3*digitDistance}; // Cursor columns for time selection in time mode
int shearModeCols2[] = {0, digitDistance, 2*digitDistance,// Cursor columns for time selection in shear mode
                        3*digitDistance, 4*digitDistance, 
                        5*digitDistance, 6*digitDistance};
float time;                                       // Duration of shear experiment [min]
float timeSeconds;                                // Duration of shear experiment [s]
float startTime;                                  // Time at start of experiment [s]
float timeElapsed;                                // Elapsed experiment time [s]
float timeRemaining;                              // Remaining experiment time [s]
float timeRemainingPre = 0;                       // Remaining experiment time of previous loop [s]
float timeStep;                                   // Time to move one step on stepper motor [mus]
float halfTime;                                   // Time to move one half step on stepper [mus]
unsigned int halfTimeInt = 0;
int timePressed = 0;                              // Time of button press [ms]
int timePrevious;                                 // Time of previous button press [ms]
int timeDelta;                                    // Time difference between button presses [ms]
float shear = 0;                                  // Total shear during shear experiment
int startupDelay = 1000;                          // Delay time of the starting screen [ms]
int minInputDelay = 100;                          // Minimal required delay between inputs [ms]
int buttonDelay = 10;                             // Button voltage sampling time [ms]
const int stepsPerRevolution = 400;               // Number of (micro-)steps per stepper rotation (half step)

//----------------------------------------------------------------
// Varia ---------------------------------------------------------
//----------------------------------------------------------------

// Create instance of stepper library
//Stepper myStepper(stepsPerRevolution, stepperPin1, stepperPin2, stepperPin3, stepperPin4);

///////////////////////////////////////////////////////////////////
// Setup function /////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

void setup() {

  // Initialize the communication method -> SPI (see DEV_Config.cpp)
  System_Init();

  // Set up serial communication (at a data rate of 9600 bits per second)
  Serial.begin(9600);

  // Set the pushbutton pins as input (retrieve signal):
  pinMode(upDownButtonPin, INPUT);
  pinMode(rightLeftButtonPin, INPUT);
  pinMode(returnButtonPin, INPUT);
  pinMode(enterButtonPin, INPUT);  

  // Set stepper motor pins as output (send signal):
  pinMode(stepperStepPin, OUTPUT);
  pinMode(stepperDirPin, OUTPUT);
  pinMode(stepperEnPin, OUTPUT);

  // Switch off stepper motor driver (until used) -> enable = LOW
  digitalWrite(stepperEnPin, HIGH);

  // Set the spinning direction clockwise (doesn't really matter for us):
  digitalWrite(stepperDirPin, HIGH);

  // Print initialization message and initialize 1.5" rgb OLED
  Serial.print(F("Initialize OLED... \r\n"));
  OLED_1in5_rgb_Init();
  Driver_Delay_ms(500); 
  eraseAll();  

  // Create a new image size
  UBYTE *BlackImage;
  Serial.print("Paint_NewImage \r\n");
  Paint_NewImage(BlackImage, OLED_1in5_RGB_WIDTH, OLED_1in5_RGB_HEIGHT, 270, BLACK);  
  Paint_SetScale(65);

  // Rotate OLED screen in alignment with control box orientation
  Paint_SetRotate(ROTATE_0); 

  // Write welcome message to OLED
  Serial.print("Drawing: StartUp Screen \r\n");
  displayText(0, 0, "Homogen.", WHITE);
  displayText(0, 40, "Shear", WHITE);
  displayText(0, 80, "Device", WHITE);

  // Delay until proceeding (to be able to read the welcome message)
  delay(startupDelay);

  // Clear display
  eraseAll(); 
}

///////////////////////////////////////////////////////////////////
// Loop function //////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

void loop() {
  //----------------------------------------------------------------
  // Check whether a button is currently being pressed and update 
  // the boolean "buttonValues" array accordingly
  //----------------------------------------------------------------

  // For details see functions defined below loop()
  buttonCheck();

  //----------------------------------------------------------------
  // Menu navigation to set up rotation speed (rpm) and time or total
  // shear for the experiment
  //----------------------------------------------------------------

  switch (status) {

    //----------------------------------------------------------------
    
    case 0: // Mode selection
      // Mode selection: TIME or TOTAL SHEAR

      // Display content
      if (firstTime[0]) {
        // Update serial monitor
        Serial.print("Drawing: Mode Selection \r\n");
        // Print text title
        displayText(0, 0, "Select", WHITE);
        displayText(0, 25, "Mode:", WHITE);
        // Print text options
        displayText(0, 60, "1. Time", WHITE);
        displayText(0, 90, "2. Shear", WHITE);
        // Add cursor line
        displayCursor(cursorCol, cursorRow, cursorCol+5*digitDistance, RED);
        // Register that first time is over
        firstTime[0] = 0;
      }


      // UP/DOWN or RIGHT/LEFT button pressed
      else if ((upPressed || rightPressed) && timeDelta > minInputDelay) {
        upPress();
      }

      // ENTER button pressed
      else if (enterPressed && timeDelta > minInputDelay) {
        enterPress();
      }

      // Exit from switch
      break;


    //----------------------------------------------------------------

    case 1: // RPM selection
      // RPM selection -> select desired rpm between 0 and max rpm

      // Change OLED pixels only for first iteration
      if (firstTime[1]) {
        // Update serial monitor
        Serial.print("Drawing: RPM Selection \r\n");
        // Print text title
        displayText(0, 0, "Select", WHITE);
        displayText(0, 25, "RPM:", WHITE);

        // TIME MODE
        if (timeMode) {
          // Print first three three digits on second row
          for (i = 0; i <= 2; i++) {   
            // Print rpm number string
            displayInteger(i*digitDistance, 60, digits[i], WHITE); 
          }
          // Display line cursor
          displayCursor(cursorCol, cursorRow, cursorCol+digitDistance/2, RED);
        }

        // SHEAR MODE
        else {
          // Print recommended rpm values on second row
          for (i = 0; i <= 1; i++) {
            for (j = 0; j <= 1; j++) {  
            // Print rpm number string
            displayInteger(shearModeCols[j], shearModeRows[i], rpmRecommended[i*2+j], WHITE); 
            }
          }

          // Display line cursor
          displayCursor(cursorCol, cursorRow, cursorCol+2*digitDistance, RED);

        }  
          
        // Reset first time bool
        firstTime[1] = 0;
      }

      // UP/DOWN button pressed
      // Change digits of rpm number 
      else if (upPressed && timeDelta > minInputDelay) {
        upPress();
      }

      // RIGHT/LEFT button pressed
      // Move cursor by one position
      else if (rightPressed && timeDelta > minInputDelay) {
        rightPress();
      }

      // RETURN button pressed
      // Return to previous screen
      else if (returnPressed && timeDelta > minInputDelay) {
        returnPress();
      }

      // ENTER button pressed
      // Store entered value and continue to next screen
      else if (enterPressed && timeDelta > minInputDelay) {
        enterPress();
      }

      break;

    //----------------------------------------------------------------
   
    case 2: // Time or total shear selection
      // Time or (total) shear selection (depending on previously selected mode)

      // Change pixel content only at first iteration
      if (firstTime[2]) {

        // TIME MODE: Time selection -> select desired time between 0 min and 99.99 min
        if (timeMode) {
          // Print text title
          displayText(0, 0, "Select", WHITE);
          displayText(0, 25, "t [min]:", WHITE);

          // Print first three three digits on second row
          for (i = 0; i <= 3; i++) {   
            if (i == 2) {
              // Print separating dot
              displayText(i*digitDistance, 60, ".", WHITE); 
            }
            else if (i > 2) {
              // Print time number string
              displayInteger(i*digitDistance, 60, digits[i-1], WHITE); 
            }
            else {
              // Print time number string
              displayInteger(i*digitDistance, 60, digits[i], WHITE); 
            }
          }

          // Display line cursor
          displayCursor(cursorCol, cursorRow, cursorCol+digitDistance/2, RED);

        }

        // SHEAR MODE: Total shear selection -> select between 0 and 1e6  
        else {
          // Print text title
          displayText(0, 0, "Select", WHITE);
          displayText(0, 25, "Shear[-]:", WHITE);

          // Print first three three digits on second row
          for (i = 0; i <= 6; i++) {   
              // Print time number string
              displayInteger(i*digitDistance*0.9, 60, digits[i], WHITE); 
          }

          // Display line cursor
          displayCursor(cursorCol*0.9, cursorRow, cursorCol*0.9+2*digitDistance/3, RED);

        }

        firstTime[2] = 0;
      }

      // UP/DOWN button pressed
      // Change digits of time number 
      if (upPressed && timeDelta > minInputDelay) {
        upPress();
      }

      // RIGHT/LEFT button pressed
      // Move cursor by one position
      else if (rightPressed && timeDelta > minInputDelay) {
        rightPress();
      }

      // ENTER button pressed
      // Store entered value and continue
      else if (enterPressed && timeDelta > minInputDelay) {
        enterPress();
      }

      // RETURN button pressed
      // Return to previous screen
      else if (returnPressed && timeDelta > minInputDelay) {
        returnPress();
      }

      break;

    //----------------------------------------------------------------

    case 3: // Review entered values
      // Review entered values for RPM, time

      // Change pixel values only at first iteration
      if (firstTime[3]) {
        // TIME MODE -> review RPM and time values
        if (timeMode) {
          // Print first line
          displayText(0, 0, "Start?", WHITE);

          // Print text rpm
          displayText(0, 30, "RPM:", WHITE);
          displayInteger(0, 55, rpm, RED);

          // Print text time
          displayText(0, 80, "t:", WHITE);
          displayFloat(0, 105, time, 1, BLUE);
          displayText(85, 105, "min", BLUE);
        }

        // SHEAR MODE -> review RPM and shear values
        else {
          // Print first line
          displayText(0, 0, "Start?", WHITE);

          // Print text rpm
          displayText(0, 30, "RPM:", WHITE);
          displayInteger(60, 30, rpm, RED);

          // Print text time
          displayText(0, 57, "Shear:", WHITE);
          displayFloat(0, 77, shear, 0, GREEN); // x old: 30

          // Print text time
          displayText(0, 105, "t:", WHITE);
          displayFloat(35, 105, time, 0, BLUE);
          displayText(85, 105, "min", BLUE);
        }

        firstTime[3] = 0;
      }    
      
      // ENTER button pressed
      // Continue
      else if (enterPressed && timeDelta > minInputDelay) {
        enterPress();
      }

      // RETURN button pressed
      else if (returnPressed && timeDelta > minInputDelay) {
        returnPress();
      }

      break;

    //----------------------------------------------------------------

    case 4: // Run experiment and show count-down timer
      // Run experiment and show count-down timer
      
      switch (timerStatus) {

        case 0:
          // Assign remaining time
          timeRemainingPre = timeSeconds;

          // Print first line
          displayText(0, 0, "Running", WHITE);
          displayText(0, 25, "...", WHITE);

          // Print remaining time
          displayFloat(0, 80, timeRemainingPre, 0, RED);
          displayText(70, 80, "s", RED);
          displayText(0, 105, "total", RED);

          // Save time duration/value until/at start of timer [s]
          startTime = millis()/1000;
          timerStatus = 1;

          // Set the enable pin low (i.e. enable motor driver)
          digitalWrite(stepperEnPin, LOW);

          delay(50);

          break;

        case 1:

          // Compute elapsed time [s]
          timeElapsed = millis()/1000 - startTime;
          // Start counter [s]
          timeRemaining = timeSeconds - timeElapsed;


          while (timeRemaining > 0) {

            // Perform first half of step
            digitalWrite(stepperStepPin, HIGH);

            // Shortest reliable microsec delay is 16383
            // Remark: halfTime has units of microseconds
            if (halfTime > 16383) {
              // To avoid overflow, halfTime = halfTime/1000 
              // for rpm == 1
              if (rpm == 1) delay(halfTimeInt);
              else delay(halfTimeInt/1000);
            }
            else delayMicroseconds(halfTimeInt);

            // Perform second half of step
            digitalWrite(stepperStepPin, LOW);

            // Shortest reliable microsec delay is 16383
            if (halfTime > 16383) {
              // To avoid overflow, halfTime = halfTime/1000 
              // only for rpm == 1
              if (rpm == 1) delay(halfTimeInt - 0.086); // Correct for calculation & sampling time (86 mus)
              else delay(halfTimeInt/1000 - 0.086); // Correct for calculation & sampling time (86 mus)
            }
            else delayMicroseconds(halfTimeInt - 86); // Correct for calculation & sampling time (86 mus)
            
            // Update elapsed time [s]
            timeElapsed = millis()/1000 - startTime;
            // Update counter [s]
            timeRemaining = timeSeconds - timeElapsed;

            /*
            // Remark: updating display takes too long -> stepper rotation lagging behind
            if (timeRemainingPre - timeRemaining >= 1 && timeRemaining >= 0) {
              // Remove old remaining time number string
              eraseFloat(0, 80, timeRemainingPre, 0);

              // Write new remaining time number string
              displayFloat(0, 80, timeRemaining, 0, RED);

              // Update previous value
              timeRemainingPre = timeRemaining;
            }
            */

            // Check if a button is pressed
            buttonCheck();

            // RETURN button pressed
            if ((returnPressed) && timeDelta > minInputDelay) {
              returnPress();

              // Set the enable pin high (i.e. disable motor driver)
              digitalWrite(stepperEnPin, HIGH);

              break;
            }

          
          }

          // Set the enable pin high (i.e. disable motor driver)
          digitalWrite(stepperEnPin, HIGH);
          
          // Switch to next screen
          timerStatus = 2;

          // Clean up mess before switching
          eraseAll();

          break;

        case 2:
          // Print message
          displayText(0, 40, "Your exp.", WHITE);
          displayText(0, 65, "is over!", WHITE);

          // Switch to next screen
          timerStatus = 3;

          break;

        case 3:
          // ENTER or RETURN button pressed
          if ((enterPressed || returnPressed) && timeDelta > minInputDelay) {
            returnPress();
          }

          break;
      }
  }
}

///////////////////////////////////////////////////////////////////
// Additional functions ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////

//----------------------------------------------------------------
//  displayText(): Print text to 1.5" OLED screen
//  Parameters:
//  xPos          ：X coordinate
//  yPos          ：Y coordinate
//  text          ：Character string to be printed
//  color         : Color of the string
//----------------------------------------------------------------

void displayText(int xPos, int yPos, const char * text, UWORD color) {
  // Additional arguments:  - font size = 20
  //                        - background color = BLACK
  Paint_DrawString_EN(xPos, yPos, text, &Font20, BLACK, color);
}


//----------------------------------------------------------------
//  eraseText(): Erase text from 1.5" OLED screen
//  Parameters:
//  xPos          ：X coordinate
//  yPos          ：Y coordinate
//  text          ：Character string to be erased
//----------------------------------------------------------------

void eraseText(int xPos, int yPos, const char * text) {
  // Additional arguments:  - font size = 20
  //                        - background color = BLACK
  //                        - text color = BLACK
  Paint_DrawString_EN(xPos, yPos, text, &Font20, BLACK, BLACK);
}

//----------------------------------------------------------------
//  displayInteger(): Print integer to 1.5" OLED screen
//  Parameters:
//  xPos          ：X coordinate
//  yPos          ：Y coordinate
//  integer       ：integer number to be displayed
//  color         : Color of the string
//----------------------------------------------------------------

void displayInteger(int xPos, int yPos, int integer, UWORD color) {
  // Additional arguments:  - font size = 20
  //                        - digit = 2 (fractional width)
  //                        - background color = BLACK
  digitString = String(integer);
  Paint_DrawNum(xPos, yPos, digitString.c_str(), &Font20, 2, color, BLACK); 
}

//----------------------------------------------------------------
//  eraseInteger(): Erase integer from 1.5" OLED screen
//  Parameters:
//  xPos          ：X coordinate
//  yPos          ：Y coordinate
//  integer       ：integer number to be displayed
//----------------------------------------------------------------

void eraseInteger(int xPos, int yPos, int integer) {
  // Additional arguments:  - font size = 20
  //                        - digit = 2 (fractional width)
  //                        - text color = BLACK
  //                        - background color = BLACK
  digitString = String(integer);
  Paint_DrawNum(xPos, yPos, digitString.c_str(), &Font20, 2, BLACK, BLACK); 
}

//----------------------------------------------------------------
//  displayFloat(): Print float to 1.5" OLED screen
//  Parameters:
//  xPos          ：X coordinate
//  yPos          ：Y coordinate
//  floatP        : floating-point number to be displayed
//  precision     : number of after-commma digits to be displayed
//  color         : Color of the string
//----------------------------------------------------------------

void displayFloat(int xPos, int yPos, float floatP, int precision, UWORD color) {
  // Additional arguments:  - font size = 20
  //                        - background color = BLACK
  digitString = String(floatP, precision);
  Paint_DrawNum(xPos, yPos, digitString.c_str(), &Font20, precision, color, BLACK); 
}

//----------------------------------------------------------------
//  eraseFloat(): Erase float from 1.5" OLED screen
//  Parameters:
//  xPos          ：X coordinate
//  yPos          ：Y coordinate
//  floatP        : floating-point number to be displayed
//  precision     : number of after-commma digits to be displayed
//----------------------------------------------------------------

void eraseFloat(int xPos, int yPos, float floatP, int precision) {
  // Additional arguments:  - font size = 20
  //                        - digit = 2 (fractional width)
  //                        - text color = BLACK
  //                        - background color = BLACK
  digitString = String(floatP, precision);
  Paint_DrawNum(xPos, yPos, digitString.c_str(), &Font20, 2, BLACK, BLACK); 
}

//----------------------------------------------------------------
//  displayCursor(): Print cursor line to 1.5" OLED screen
//  Parameters:
//  xPosLeft      ：X coordinate (left)
//  xPosRight     : X coordinate (right)
//  yPos          ：Y coordinate
//  color         : Color of the string
//----------------------------------------------------------------

void displayCursor(int xPosLeft, int yPos, int xPosRight, UWORD color) {
  // Additional arguments:  - line width = DOT_PIXEL_2X2 (2x2 px)
  //                        - line style = LINE_STYLE_SOLID (solid line)
  Paint_DrawLine(xPosLeft, yPos, xPosRight, yPos, color, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
}

//----------------------------------------------------------------
//  eraseCursor(): Remove cursor line from 1.5" OLED screen
//  Parameters:
//  xPosLeft      ：X coordinate (left)
//  xPosRight     : X coordinate (right)
//  yPos          ：Y coordinate
//----------------------------------------------------------------

void eraseCursor(int xPosLeft, int yPos, int xPosRight) {
  // Additional arguments:  - line color = RED
  //                        - line width = DOT_PIXEL_2X2 (2x2 px)
  //                        - line style = LINE_STYLE_SOLID (solid line)
  Paint_DrawLine(xPosLeft, yPos, xPosRight, yPos, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
}

//----------------------------------------------------------------
//  eraseAll(): Remove all content from 1.5" OLED screen
//----------------------------------------------------------------
void eraseAll() {
  OLED_1in5_rgb_Clear();
}

//----------------------------------------------------------------
//  buttonCheck(): Check whether button is pressed
//----------------------------------------------------------------

int buttonCheck() {
  // Reset the buttonPressed bool to zero
  buttonPressed = 0;

  // Read the state of each pushbutton:
  buttonValues[0] = digitalRead(returnButtonPin);
  buttonValues[1] = digitalRead(upDownButtonPin);
  buttonValues[2] = digitalRead(rightLeftButtonPin);
  buttonValues[3] = digitalRead(enterButtonPin);

  // Assign values to more intuitive variables
  returnPressed = buttonValues[0];
  upPressed = buttonValues[1];
  rightPressed = buttonValues[2];
  enterPressed = buttonValues[3];

  // Update tracker variables if a button was pressed
  for (i = 0; i < buttonElements; i++) {
    if (buttonValues[i] != 0) {
      buttonPressed = 1;
      timePrevious = timePressed;
      timePressed = millis();
      timeDelta = timePressed - timePrevious;
      break;
    }
  }
}

//----------------------------------------------------------------
//  enterPress(): Advance in menu structure after enter pressed
//----------------------------------------------------------------

void enterPress() {
  
  if (status == 0) {
    // Set time or shear mode based on cursor position
    if (cursorRow == 60 + cursorDistance) {
      timeMode = 1;     // Time mode selected
    }

    else timeMode = 0;  // Shear mode selected

    // Switch to next screen
    status = 1;

    // Reset cursor position
    cursorRow = 60 + cursorDistance;
    cursorCol = 0;

    // Clean up mess before switching
    eraseAll();
  }

  else if (status == 1) {
    // TIME MODE
    if (timeMode) {
      // Calculate RPM based on selected digits
      rpm = digits[0] * 100 + digits[1] * 10 + digits[2];
      // Set new digits for time selection
      digits[0] = 0;
      digits[1] = 1;
      digits[2] = 0; 
    }
    // SHEAR MODE
    else {
      for (i = 0; i <= 1; i++) {
        for (j = 0; j <= 1; j++) {
          if (cursorRow == shearModeRows[i] + cursorDistance && 
              cursorCol == shearModeCols[j]) {
            rpm = rpmRecommended[i*2+j];
          }
        }
      }
      // Save shear rate corresponding to selected rpm
      for (i = 0; i <= 3; i++) {
        if (rpm == rpmRecommended[i]) {
          shearRate = shearRecommended[i];
        }
      }

      // Set new digits for shear selection
      digits[0] = 1;
      digits[1] = 0;
      digits[2] = 0; 
    }

    // Calculate time (in mus) per step corresponding to entered rpm value
    timeStep = 1000000.0 / (1.0 * rpm * stepsPerRevolution / 60.0);
    halfTime = timeStep / 2.0;
    
    // Multiply by 0.001 since max value unsigned int = 65'535
    // and at rpm == 1 we get 75'000 -> otherwise overflow
    if (rpm == 1) halfTimeInt = 0.001 * halfTime;
    else halfTimeInt = halfTime;

    Serial.println(timeStep);
    Serial.println(halfTime);
    Serial.println(halfTimeInt);

    // Switch to next screen
    status = 2;

    // Reset cursor position
    cursorRow = 60 + cursorDistance;
    cursorCol = 0;

    // Clean up mess before switching
    eraseAll();
  }

  else if (status == 2) {
    // TIME MODE
    if (timeMode) {
      // Compose time based on selected digits
      time = digits[0] * 10 + digits[1] + digits[2] * 0.1;
      timeSeconds = digits[0] * 600 + digits[1] * 60 + digits[2] * 6;
  
      // Reset digits
      for (i = 0; i <= 6; i++) {
        if (i == 0) {
          digits[i] = 1;
        }
        else  digits[i] = 0;
      }

      // Switch to next screen
      status = 3;

      // Reset cursor position
      cursorRow = 60 + cursorDistance;
      cursorCol = 0;

      // Clean up mess before switching
      eraseAll();
    }
    // SHEAR MODE
    else {
      // Compose total shear based on selected digits
      shear = digits[0] * 1000000 + digits[1] * 100000 
              + digits[2] * 10000 + digits[3] * 1000
              + digits[4] * 100 + digits[5] * 10
              + digits[6];

      // Calculate time based on total shear & shear rate
      timeSeconds = shear / shearRate;
      time = timeSeconds / 60;

      // Reset digits
      for (i = 0; i <= 6; i++) {
        if (i == 0) {
          digits[i] = 1;
        }
        else  digits[i] = 0;
      }

      // Switch to next screen
      status = 3;

      // Reset cursor position
      cursorRow = 60 + cursorDistance;
      cursorCol = 0;

      // Clean up mess before switching
      eraseAll();
    }
  }

  else if (status == 3) {
    // Reset digits
    for (i = 0; i <= 6; i++) {
      if (i == 0) {
        digits[i] = 1;
      }
      else  digits[i] = 0;
    }

    // Switch to next screen
    status = 4;

    // Reset cursor position
    cursorRow = 60 + cursorDistance;
    cursorCol = 0;

    // Clean up mess before switching
    eraseAll();
  }
      
}

//----------------------------------------------------------------
//  upPress(): Switch cursor position or current digit upwards
//----------------------------------------------------------------

void upPress() {

  // Mode selection menu
  if (status == 0) {
    // Switch between options
    if (cursorRow == 60 + cursorDistance) {
      // Remove previous cursor
      eraseCursor(cursorCol, cursorRow, cursorCol+5*digitDistance);
      // Update cursor row (= vertical position)
      cursorRow = 90 + cursorDistance;
      // Display new cursor
      displayCursor(cursorCol, cursorRow, cursorCol+5.5*digitDistance, RED);
    }

    else {
      // Remove previous cursor
      eraseCursor(cursorCol, cursorRow, cursorCol+5.5*digitDistance);
      // Update cursor row (= vertical position)
      cursorRow = 60 + cursorDistance;
      // Display new cursor
      displayCursor(cursorCol, cursorRow, cursorCol+5*digitDistance, RED);
    }
  }

  // RPM selection
  else if (status == 1) {
    // TIME MODE
    if (timeMode) {
      for (i = 0; i <= 2; i++) {
        if (cursorCol == i*digitDistance) {
          // Remove previous digit
          eraseInteger(i*digitDistance, 60, digits[i]); 
          
          // Update digit values
          if (digits[i] < 9) {
            if (digits[0] == 4) {
              // Set 400 rpm as max. value (faster impossible w/o acceleration profile)
              if (i == 0) {
                digits[0] -= 4;
              }
              else {
                digits[i] = 0;
              }
            }
            else if (digits[0] == 3) {
              if (i == 0) {
                digits[0] += 1;
                
                eraseInteger(1*digitDistance, 60, digits[1]); 
                digits[1] = 0;
                displayInteger(1*digitDistance, 60, digits[1], WHITE);

                eraseInteger(2*digitDistance, 60, digits[2]); 
                digits[2] = 0;
                displayInteger(2*digitDistance, 60, digits[2], WHITE);
              }
              else digits[i] += 1;
            }
            else digits[i] += 1;
          }
          else digits[i] -= 9;
          
          // Display new digit
          displayInteger(i*digitDistance, 60, digits[i], WHITE);
        }
      }
    }
    // SHEAR MODE
    else {
      // Remove previous line cursor
      eraseCursor(cursorCol, cursorRow, cursorCol+2*digitDistance);

      // Update cursor position
      if (cursorRow == shearModeRows[0] + cursorDistance) {
        cursorRow = shearModeRows[1] + cursorDistance;
      }
      else {
        cursorRow = shearModeRows[0] + cursorDistance;
      }

      // Draw new line cursor
      displayCursor(cursorCol, cursorRow, cursorCol+2*digitDistance, RED);
    }
  }

  // Time or total shear selection
  else if (status == 2) {
    // TIME MODE
    if (timeMode) {
      for (i = 0; i <= 2; i++) {
        if (cursorCol == timeModeCols[i]) {

          if (i == 2) {
            // Remove old time number string (after separating dot)
            eraseInteger((i+1)*digitDistance, 60, digits[i]);  
          }
          
          else {
            // Remove old time number string (before separating dot)
            eraseInteger(i*digitDistance, 60, digits[i]);    
          }

          // Update digit values
          if (digits[i] < 9) {
            digits[i] += 1;
          }
          else digits[i] -= 9;

          if (i == 2) {
            // Display new time number string (after separating dot)
            displayInteger((i+1)*digitDistance, 60, digits[i], WHITE);  
          }
          
          else {
            // Display new time number string (before separating dot)
            displayInteger(i*digitDistance, 60, digits[i], WHITE);    
          }
        }
      }
    }
    // SHEAR MODE
    else {
      for (i = 0; i <= 6; i++) {
        if (cursorCol == shearModeCols2[i]) {

          // Remove old time number string
          eraseInteger(i*digitDistance*0.9, 60, digits[i]); 

          // Increment current digit
          if (digits[i] < 9) {
            digits[i] += 1;
          }
          else digits[i] -= 9;

          // Display new time number string
          displayInteger(i*digitDistance*0.9, 60, digits[i], WHITE); 

          break;
        }
      }
      
    }
  }
  
  // Add delay for stability
  delay(minInputDelay);

}

//----------------------------------------------------------------
//  rightPress(): Switch cursor position to the right
//----------------------------------------------------------------

void rightPress() {

  // RPM selection
  if (status == 1) {
    // TIME MODE
    if (timeMode) {
      // Remove previous line cursor
      eraseCursor(cursorCol, cursorRow, cursorCol+digitDistance/2);

      // Update cursor column (= horizontal position)
      if (cursorCol < 2*digitDistance) {
        cursorCol += digitDistance;
      }
      else if (cursorCol = 2*digitDistance) {
        cursorCol = 0;
      }

      // Draw new line cursor
      displayCursor(cursorCol, cursorRow, cursorCol+digitDistance/2, RED);
    }
    // SHEAR MODE
    else {
      // Remove previous line cursor
      eraseCursor(cursorCol, cursorRow, cursorCol+2*digitDistance);

      // Update cursor column (= horizontal position)
      if (cursorCol == shearModeCols[0]) {
        cursorCol = shearModeCols[1];
      }
      else {
        cursorCol = shearModeCols[0];
      }

      // Draw new line cursor
      displayCursor(cursorCol, cursorRow, cursorCol+2*digitDistance, RED);
    }
  }

  else if (status == 2) {
    // TIME MODE
    if (timeMode) {
      for (i = 0; i <= 3; i++) {
        if (cursorCol == timeModeCols[i]) {

          // Display line cursor
          eraseCursor(cursorCol, cursorRow, cursorCol+digitDistance/2);
          
          // Update cursor position
          if (i == 2) {
            cursorCol = 0;
          }
          else if (i == 1) {
            cursorCol += 2*digitDistance;
          }
          else cursorCol += digitDistance;

          // Display line cursor
          displayCursor(cursorCol, cursorRow, cursorCol+digitDistance/2, RED);

          break;
        }
      }
    }
    // SHEAR MODE
    else {
      for (i = 0; i<= 6; i++) {
        if (cursorCol == shearModeCols2[i]) { 

          // Remove old line cursor
          eraseCursor(cursorCol*0.9, cursorRow, cursorCol*0.9+2*digitDistance/3);

          // Update cursor position
          if (i == 6) {
            cursorCol = shearModeCols2[0];
          }
          else cursorCol = shearModeCols2[i+1] ;

          // Display line cursor
          displayCursor(cursorCol*0.9, cursorRow, cursorCol*0.9+2*digitDistance/3, RED);

          break;
        }
      }
    }
  }

  // Add delay for stability
  delay(minInputDelay);
}

//----------------------------------------------------------------
//  returnPress(): Advance in menu structure after enter pressed
//----------------------------------------------------------------

void returnPress() {
 
  // RPM selection
  if (status == 1) {
    // Reset cursor position
    cursorRow = 60 + cursorDistance;
    cursorCol = 0;

    // Clean up mess before return
    eraseAll();

    // Switch to previous screen
    status = 0;

    // Reset first time visit for all screens up to current
    firstTime[0] = 1;
    firstTime[1] = 1;
  }

  // Time or shear selection
  else if (status == 2) {
    // Reset cursor position
    cursorRow = 60 + cursorDistance;
    cursorCol = 0;

    // Clean up mess before return
    eraseAll();

    // Switch to previous screen
    status = 1;

    // Reset first time visit for all screens up to current
    firstTime[0] = 1;
    firstTime[1] = 1;
    firstTime[2] = 1;

    // Reset digits
    digits[0] = 1;
    digits[1] = 0;
    digits[2] = 0;
  }

  // Review values
  else if (status == 3) {
    // Reset digits
    for (i = 0; i <= 6; i++) {
      if (i == 0 && timeMode == 0) {
        digits[i] = 1;
      }
      else if (i == 1 && timeMode == 1) {
        digits[i] = 1;
      }
      else  digits[i] = 0;
    }

    // Reset cursor position
    cursorRow = 60 + cursorDistance;
    cursorCol = 0;

    // Clean up mess before return
    eraseAll();

    // Switch to previous screen
    status = 2;

    // Reset first time visit for all screens up to current
    firstTime[0] = 1;
    firstTime[1] = 1;
    firstTime[2] = 1;
    firstTime[3] = 1;
  }

  // Run experiment and timer
  else if (status == 4) {
    // Reset cursor position
    cursorRow = 60 + cursorDistance;
    cursorCol = 0;

    // Clean up mess before return
    eraseAll();

    // Reset first time visit for all screens up to current
    firstTime[0] = 1;
    firstTime[1] = 1;
    firstTime[2] = 1;
    firstTime[3] = 1;
    firstTime[4] = 1;

    if (timerStatus == 3) {
      // Reset status
      status = 0;
    }
    else status = 3;

    // Reset timerStatus
    timerStatus = 0;
  }

}
