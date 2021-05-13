close all;
clear;

# This file automatically plots the result saved from a log from 
# arduino sketch "Motor11_PID_Tuning_SL1" that output m array automatically
# The result file can hold any number of result series

# Result file name to read from
file_name = "motor3_result5";
xsize_of_plot = 800;
ysize_of_plot = 600;

eval(file_name);
names = who("result_*");

# Legend String Processing
legend_strings = strrep (names,"result_","")
legend_strings = strrep (legend_strings,"_",", ")
legend_strings = strrep (legend_strings,"p",".")

# File Name Output
name_of_plot_1 = strcat (file_name, "_err.png")
name_of_plot_2 = strcat (file_name, "_out.png")
name_of_plot_3 = strcat (file_name, "_pos.png")


function plot_error(m)
  m = [m(:,1), m(:,3)-m(:,2)];
  plot(m(:, 1)/1000,m(:, 2));
endfunction
function plot_ouput(m)
  plot(m(:, 1)/1000,m(:, 4));
endfunction

# Plot 1 Error/Time
figure(1, 'position',[0,0,xsize_of_plot,ysize_of_plot]);
hold on
for i = 1:length(names)
  m = eval(names{i});
  plot_error(m);
end
hold off
legend (legend_strings ,"location", "southeast");
grid on

axis("tight")
xlabel ("Time [ms]")
ylabel ("Error [step]");
title("PID Error with Trapezoidal Motion Profile")
drawnow
print(name_of_plot_1,["-S",num2str(xsize_of_plot),",",num2str(ysize_of_plot)],"-F:10")

# Plot 2 Output/Time
figure(2, 'position',[0,0,xsize_of_plot,ysize_of_plot]);
hold on
for i = 1:length(names)
  m = eval(names{i})
  plot_ouput(m)
end
hold off
legend (legend_strings ,"location", "southeast");
grid on

axis("tight")
ylim ([-1, 1]);
xlabel ("Time [ms]");
ylabel ("Power Output [Ratio of full speed]");
title("PID Output with Trapezoidal Motion Profile");
drawnow
print(name_of_plot_2,["-S",num2str(xsize_of_plot),",",num2str(ysize_of_plot)],"-F:10")

# Plot 3 Position/Time
figure(3, 'position',[0,0,xsize_of_plot,ysize_of_plot]);
hold on
for i = 1:length(names)
  m = eval(names{i})
  plot(m(:, 1)/1000,m(:, 3));
end
hold off
legend (legend_strings ,"location", "southeast");
grid on

axis("tight")
xlabel ("Time [ms]")
ylabel ("Measured Position [step]");
title("Position over Time with Trapezoidal Motion Profile")
drawnow
print(name_of_plot_3,["-S",num2str(xsize_of_plot),",",num2str(ysize_of_plot)],"-F:10")
close all