This is CANOpen Node library for Arduino Due base on example of HamedJafarzadeh (https://github.com/HamedJafarzadeh/CanOpenSTM32)
I converted from STM32CubeIDE to Microchip Studio and Arduino IDE to build the lirary.
Clock to config CAN was fixed 84000000.
I use DueTimer v1.4.7 to create a millisecond tick in GetTick variable.
OD.h and OD.c are generated by Object Dictionary Editor v4.0-51-g2d9b1ad to CANOpenNode folder.
If errors appear when exporting OD.h and OD.c please check the Access SDO artribute of *.xdp, eds,.. files in Object Dictionary Editor software

Because there are my first published Arduino library so it have some untidy code. ^^

This project is licensed under the terms of the MIT license.