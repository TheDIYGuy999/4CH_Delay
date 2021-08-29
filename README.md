# This is an Arduino Pro Mini 3.3V / 8MHz based RC servo ramp / delay generator
## Features:
- 4 RC servo PWM inputs and outputs (can be enhanced)
- Reads the inputs, adds an acceleration / deceleration ramp as specified and generates new output signals
- Used to make RC vehicle movements smooth and more realistic
- I'm using it in my KABOLITE K336 hydraulic excavator
- You can also use it as a channel mixer by changing the sketch

See: https://www.youtube.com/playlist?list=PLGO5EJJClJBCjIvu8frS7LrEU3H2Yz_so

New in V 1.0:
- Initial commit

## Wiring

- See pictures and comments in sketch
- Basically it's wired in between the receiver output and the servo / ESC
- The Pro Mini is supplied by the servo wires via the GND and RAW pins
- WARNING! Most modern RC electronics devices are using a signal voltage of 3.3V, so you have to use the 3.3v / 8MHz version!

![](https://github.com/TheDIYGuy999/4CH_Delay/blob/master/pictures/wiring.jpg)
![](https://github.com/TheDIYGuy999/4CH_Delay/blob/master/pictures/excavator.jpg)

(c) 2021 TheDIYGuy999
