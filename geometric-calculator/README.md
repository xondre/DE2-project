# Digital Electronics 2 - Project 1

## Geometric Calculator

This project was created in the spirit of showing an example how could the analog joy-stick, rotary encoder and Digilent PmodCLP LCD module
be combined into single working system on the platform of Arduino UNO. The project is a calculator which can compute surface and perimeter
of rectangle or circle. Moving the joystick up or down changes the result type (rectangle or circle) and moving it left or right switches
between the input variables. Encoder is used to change value of the selected input variable.

### Team members

* Adam Ondrejka
* Dominik Vaško

## Hardware description

Most of the pins on each component can be connected directly to Arduino UNO, with exception of ground and power supply pins, which first need to go
to some kind of bus, such as breadboard, and then to Arduino. In this project, the display module works in 4-bit mode and therefore only the lower
portion of J1 connector is wired. Because the rotary encoder utilizes external interrupt INT0, it has to be connected to the pin D2. Similarly, outputs
from the joystick have to be connected to pins A0 and A1 because the code is written to use ADC channels 0 and 1.

![Schematic of wiring](schematic.png)

## Simulation

This project was developed mainly using the online Arduino simulator [wokwi](https://wokwi.com/). Testing on the real board was done only as a last step.
Below is the link for the simulation project.
The process of simulating was as follows:
* compile the code and create HEX file
* go to the [project simulation webpage](https://wokwi.com/projects/350052781060522578)
* click into the "source_code.ino" tab in the top left part of the screen
* click anywhere into the empty code
* press F1 and type "HEX"
* select "Load HEX file and Start Simulation"
* select the HEX file for uploading
* simulation starts


## Software description

![main](main.png)
![INT0_vect](INT0.png)
![ADC](ADC.png)
![get_joystick](get_joystick.png)
![update_input](update_input.png)
![update_result](update_result.png)
![add_decimal](add_decimal.png)

## Video

[Short video showing the working project](https://youtu.be/gwnjN7g5l00)

## References

1. [Digilent PmodCLP LCD module information](https://digilent.com/reference/pmod/pmodclp/start)
2. [Atmega328P datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf)
3. [online simulator wokwi](https://wokwi.com/)
4. [Tool for creating custom character definitons](https://omerk.github.io/lcdchargen/)
