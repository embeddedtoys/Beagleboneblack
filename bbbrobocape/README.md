
Repository for bbbrobocape support software. 

component_pdf - datasheet pdf of components used. The pdf's for resistors and cpacitors were left out.
workspacebbb - eclipse project workspace folder for robocape.
robocape_BOM_RevA - BOM in pdf form.
robocapedts - device trees for the spi ports and gpio pins used.
RoboGUISW - WIN7 GUI for testing board. Was built using .NET4.5.
schematics - schematics and assembly drawing for robocape PCB.

The software Allows user to test functionality of the various sections of robocape. The code is a collection
of open source lib's already out there for beaglebone black. The eclipse project was based on Derek Malloys
nice tutorial from his website. He puts out some great tutorials. The PWM module is based on Saadahmad's C++
lib. Works great the pwm used are PWM1A PWM1B PWM2A and PWM2B. The motor control section is ST Micro's firmware 
code converted to a class. The network class used is netlink. Seems to work ok. Keep in mind you can use all the software
or cut out what you don't like or just write your own from scratch, that's what's great about free software.

The L6470's use The SPI1 interface while the Digital I/O MCP23S17 and MAX1300 ADC use SPI0. This
was done so seperate threads can access the motors separately from the digital I/O and ADC.

Included in the eclipse workspace is pthread lib. This had to be put in the cross compiler lib folder. You may need
to do the same if you crosscompile. Some people compile directly on bbb so you may not need it.

The sw was compiled and tested on Angstrom. It should be ok for Ubuntu as well. The cross compiler used
is arm-linux-gnueabi. If you use arm-linux-gnueabihf you will need to change the eclipse project settings to 
point to it.

for loading the capes for the board use
echo BB-SPI1-01 > /sys/devices/bone_capemgr.*/slots
echo BB-SPI0-01 > /sys/devices/bone_capemgr.*/slots
echo RoboCapeGpio > /sys/devices/bone_capemgr.*/slots

There is a script in the eclispe folder to do this one step. The PWM firmware loads via code. A nice feature
Saadahmad included in his PWM lib.

The gui sw includes a pdf viewer for viewing the schematics or data sheets. There is also a spread sheet control
for any motor calculations needed. The L6470 is a little complicated at first but once you experiment a little it's
not to bad. It has a nice command set that unloads the processor of a lot of work.