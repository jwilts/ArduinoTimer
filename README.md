CubKars
Timer for Cub Car Tracking (Pinewood Derby cars)

Pinewood Derby Race Timer using Arduino NANO
This project is a simple system for running a Pinewood Derby (Cubcar) race using an Arduino NANO. The project supports either a 3-lane or 4-lane track, displaying race results and times on a 4x20 I2C LCD screen and indicating the winner with a bank of LEDs.

Features
Lane Support: Supports both 3-lane and 4-lane tracks.
LED Winner Indicator: A bank of LEDs controlled by two shift registers indicates the race winner and positions.
LCD Display: Displays individual race times and instructions using a 4x20 I2C LCD.
LDR Finish Line Sensors: Light Detecting Resistors (LDRs) at the finish line trigger the race timers as cars cross the finish line.
Tuning Routine: Automatically adjusts to varying light conditions at the start of each race to ensure accurate timing.
Debug Mode: Outputs verbose information to the Serial Monitor for troubleshooting.
Components
Arduino NANO: The microcontroller that powers the project.
Shift Registers: Two shift registers reduce the number of pins needed to control up to 16 LEDs.
4x20 I2C LCD Display: Displays race information and instructions.
Light Detecting Resistors (LDRs): Embedded at the finish line to detect cars.
External Pulldown Resistors: Used with the LDRs for more accurate sensing.
Power Supply
This project requires more power than a standard USB port can provide. A Buck converter powered by a 9V power supply is used to ensure reliable operation, and it also powers an always-on LED strip that provides consistent lighting to the finish line.

Circuit Diagram
LDR Circuit (with External Pulldown Resistors)
3V3 -- LDR -- A0 (Nano) -- 10K Resistor -- GND
Same pattern is used for other LDRs and switches.

Pin Connections
Arduino Pin	Function
D0 (RX)	Do not connect anything (upload issue)
D1 (TX)	Do not connect anything (upload issue)
D2 (INT0)	Push button for timer (TimerButton)
D3 (INT1)	Start timer switch (StartButton)
D8	Shift Register - Latch Pin (12)
D11 (MOSI)	Shift Register - Data Pin (14)
D12 (MISO)	Shift Register - SH_Clock Pin (11)
A0-A3	Lane LDR inputs
A4 (SDA)	I2C SDA for LCD
A5 (SCL)	I2C SCL for LCD
Setup
Power: Ensure the Arduino NANO is connected to a reliable power supply using a Buck converter and a 9V source.
LCD Setup: The 4x20 I2C LCD should be connected to the I2C interface (SDA and SCL pins).
LDR Sensors: Place LDRs at the finish line to detect car crossing, ensuring a consistent light source is overhead.
Shift Register: Use two shift registers to control the bank of LEDs for displaying the winner and positions.
Tuning Routine
At each startup, the system will run a tuning routine to adjust for the current lighting conditions:

The system measures ambient light with no car on the track.
Then, it measures the light with cars shading the LDR sensors.
Thresholds are automatically calculated to accurately detect when a car crosses the finish line.
Debug Mode
To enable verbose output for troubleshooting, toggle the DEBUG macro in the code:

#define DEBUG 1
By default, DEBUG is disabled:

#define DEBUG 0
When debugging is enabled, detailed messages are sent to the Serial Monitor.

Author
Jeff Wilts
Version 1.0 — Initial Build (May 2023)
License
This project is licensed under the MIT License. Feel free to use, modify, and distribute.

Feel free to clone the repository and start building your own Pinewood Derby race timer!
