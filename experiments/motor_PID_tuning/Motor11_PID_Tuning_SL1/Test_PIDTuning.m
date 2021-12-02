clear
pkg load instrument-control

# Opens serial port ttyUSB1 with baudrate of 115200 (config defaults to 8-N-1)
s1 = serial("COM6", 115200) ;
filepath = 'data\' 

# Wait a few seconds for Arduino to boot up
pause(2);
# Flush input and output buffers
srl_flush(s1);

# Command to send to Arduino
srl_write(s1, "v4736\n") ;
srl_write(s1, "a10000\n") ;
srl_write(s1, "kp0.02\n");
srl_write(s1, "ki0.1\n") ;
srl_write(s1, "kd0.0002\n") ;

pause(1)
param_key = 'ki'
srl_flush(s1);
value_target_to_test = 0.1
values_to_test = [0.5, 0.75, 1.0, 1.5, 2] * value_target_to_test
multiple_results = {}

for i = 1:size(values_to_test,2)
  value = values_to_test(i);
  # Command to send to Arduino
  srl_write(s1, [param_key mat2str(value) "\n"]) ;
  # Command to send to Arduino
  srl_write(s1, "h\n") ;
  # Command to send to Arduino
  srl_write(s1, "g6000\n") ;

  readings = []
  for j = 1:3000
    return_string = ReadToTermination(s1, 10);
    if return_string(1) == '>'
      reading = textscan (return_string, ">%d,%d,%d,%d,%d,%d");
      readings = [readings ; reading];
      # End condition when motion stops, checking the second bit.
      if bitget(reading{1,1},2) == 0
        break 
      endif
    endif
  endfor

  m = cell2mat(readings);
  multiple_results{i} = m;
  csvwrite([filepath "result_" param_key "_" mat2str(value) ".csv"], m);
  pause (1);
  
endfor

# Close serial connection
clear('s1')
