figure(1, 'position',[10,10,800,600]);
hold on
legend_str = {};
for i=1:numel(values_to_test)
  ## Extracat file name
  data = multiple_results{i};
    
  time = data(:,2) ;
  error = data(:,3) - data(:,4) ;

  plot (time, error);
  legend_str = cat(2,legend_str,mat2str(values_to_test(i)));
endfor
hold off

# Axis Limit Configuration
axis("tight")
legend(legend_str, "location", "southwest");

# Labels and Title
xlabel ("Time [ms]")
ylabel ("Error [Nm]");
title(["Tune run 8 - Tuning ki"], 'Interpreter', 'none')
grid on

print(1, ["data/tune_8.jpg"], "-djpg", "-S1200,800")


