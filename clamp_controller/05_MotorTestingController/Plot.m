all_file_names = glob('result_ki_*.csv')

figure(1, 'position',[10,10,800,600]);
hold on

for i=1:numel(all_file_names)
  ## Extract file name
  [dir, file_name, ext] = fileparts (all_file_names {i})
  data = csvread ([file_name ".csv"])
  

  
  time = data(:,2) 
  error = data(:,3) - data(:,4) 

  plot (time, error)
  
endfor
hold off

# Axis Limit Configuration
axis("tight")
legend(all_file_names)
# Labels and Title
xlabel ("Time [ms]")
ylabel ("Error [Nm]");
title(["Error with different Ki"], 'Interpreter', 'none')
grid on

#print(1, ["all_torque_plot.jpg"], "-djpg", "-S1200,800")


