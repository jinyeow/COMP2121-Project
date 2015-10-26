# COMP2121 Project - UNSW Semester 2 2015

## Lift Emulator

Develop a lift emulator system using AVR.

### Requirements:
1. Lift travels up and down 10 floors, numbered 0-9.
2. Keypad buttons 0-9 represent the floor select and call buttons.
3. Multiple requests may be given and should be taken. These calls should be
   performed in order
4. Lift takes *2 seconds* to travel between floors.
5. Lift open-close process:
  * Open the door (1 second)
  * Leave the door open (3 seconds)
  * Close door (1 second)
6. Right Push Button (PB0) is the 'Close' button. Operates as follows:
  * If PB0 is pushed while the door is open, start closing the door immediately.
  * The 'Close' button will *NOT* cancel the opening sequence (just the waiting sequence).
7. '*' is the emergency button. Pushing this stops lift operations. The lift
  should immediately head toward the ground floor (Floor 0) and then open & close
  so people can get out. Then the lift should halt. The LCD then shows this
  message:
      "Emergency
       Call 000"
  The strobe LED should blink several times/second to denote the alarm.
  The lift resumes normal operations again when the '*' is pressed again.
8. The LEDs should indicate the direction the lift is moving and the status
  of the door (i.e. open, close, moving).
9. Display current floor number on the LCD.
10. To indicate the door is opening or closing, the motor should spin at FULL
  SPEED.

### Advanced Features:
Use left push button (PB1) as the 'Open' button inside the lift. It should
operate as follows:
1. If the Open button is pushed while the lift is stopped at a floor, it should
open according to the procedure in Point 5.
2. If the Open button is pushed while the lift is closing, the door should stop
closing, re-open and continue operating as in Point 5.
3. If the Open button is held down while the door is open, the door should
remain open until the button is released.
4. The Open button will *NOT* function while the lift is moving between floors.

### Other Documentation for submission:
* User Manual
* Design Manual
