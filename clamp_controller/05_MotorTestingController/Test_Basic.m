clear
pkg load instrument-control

filename = ['result_' (strftime ("%Y-%m-%d_%H-%M-%S", localtime (time ())))]

# Opens serial port ttyUSB1 with baudrate of 115200 (config defaults to 8-N-1)
s1 = serial("COM3", 115200) ;
# Wait a few seconds for Arduino to boot up
pause(2);
# Flush input and output buffers
srl_flush(s1);

# Command to send to Arduino

srl_write(s1, "v1200\n") ;
srl_write(s1, "kp0.01\n");
srl_write(s1, "ki0.04\n") ;
srl_write(s1, "kd0.0002\n") ;
srl_write(s1, "h\n") ;
pause(1)

# Notes to be written in plot
notes = '(20201118 1-13 motor, 15 hole, long screw. run 1. v1200 g-36600 kp0.01 ki0.04 kd0.0002)'
#notes = '(Dry run after oil. v2000 g36600 kp0.01 ki0.04 kd0.0002)'

# Movement Command to send to Arduino
# g-18300 - one turn
# g-36600 - two turns
# g-73200 - three turns
srl_flush(s1);
pause(1)
srl_write(s1, "g-36600\n") ;

readings = [];
while(1)
  return_string = ReadToTermination(s1, 10);
  if return_string(1) == '>'
    reading = textscan (return_string, ">%d,%d,%d,%d,%d,%d");
    readings = [readings ; reading];
    # End condition when motion stops, checking the second bit.
    if bitget(reading{1,1},2) == 0
      break 
    endif
  endif
endwhile

# Data in matrix
result = cell2mat(readings);
# Data to be written to CSV with header
column_headers = {"status_code" "time" "position" "target" "power" "batt"}
csvwrite([filename '.csv'] , result);
  
# Close serial connection
clear('s1')

# Plot Power and Error Graph

#hold on
 
time = result(:,2) ;
pos = result(:,3);
error = result(:,3) - result(:,4) ;
power = result(:,5);
close
f = figure(1, 'position',[10,10,1800,600]);


subplot (1, 3, 1);
plot (time, pos, '-k');
axis("tight")
xlabel ("Time [ms]")
ylabel ("Position [step]")
title([strrep(filename, '_', '\_') " - Position"])

subplot (1, 3, 2);
hold on;
plot (time, power, 'b');
area (time, power, "FaceColor", "blue");
hold off;
axis("tight")
ylim([-100,100])
xlabel ("Time [ms]")
ylabel ("Power [percent]")
title([strrep(filename, '_', '\_') " - Power"])

subplot (1, 3, 3);
hold on;
plot (time, error, 'r');
area (time, error, "FaceColor", "red");
axis("tight")
ylim([-200,200])
hold off;
xlabel ("Time [ms]")
ylabel ("Error [step]")
title([strrep(filename, '_', '\_') " - Error"])

text(50,180,notes)
print (f, [filename ".jpg"], '-S1800,600');
