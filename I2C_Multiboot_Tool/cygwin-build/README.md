Cygwin build of Multiboot Tool for Windows

For use with the Arduino I2C tool if programming MappyDots under Windows. Does not work with I2C devices on Windows. - https://github.com/MappyDot/Programming/tree/master/Arduino_I2C_Adapter

If you get the error "error 22 from tcgetattr", open the device in a terminal application like Putty first, then run again.

Run with the following command for COM5: 

./twiboot-arduino.exe -a 0x08 -d /dev/ttyS4 

Where the USB serial device is the Windows COM number - 1. For for COM5 it's /dev/ttyS4. More options are available with the -h command.

You need the cygwin1.dll file in the same directory as the exe.