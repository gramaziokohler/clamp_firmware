clear
pkg load instrument-control

# Opens serial port ttyUSB1 with baudrate of 115200 (config defaults to 8-N-1)
s1 = serial("COM3", 115200) 
# Wait a few seconds for Arduino to boot up
pause(2)
# Flush input and output buffers
srl_flush(s1);

# Command to send to Arduino
srl_write(s1, "g1000\n") 

record = []
for i = 1:10
  return_string = ReadToTermination(s1, 13)
endfor

clear('s1')