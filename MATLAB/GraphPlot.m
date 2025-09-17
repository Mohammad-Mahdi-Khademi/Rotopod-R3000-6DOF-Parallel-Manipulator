clc;
clear;
close all;

%-------------------------------------------------------------------------%

filename = 'Graph.csv'; 

data = readtable(filename, 'VariableNamingRule', 'preserve');

time = data{:, 1};
Joint1 = data{:, 2}; 
Joint2 = data{:, 3}; 
Joint3 = data{:, 4}; 
Joint4 = data{:, 5}; 
Joint5 = data{:, 6}; 
Joint6 = data{:, 7}; 

figure;

plot(time, Joint1, '-'); 
hold on; 
plot(time, Joint2, '-'); 
plot(time, Joint3, '-'); 
plot(time, Joint4, '-'); 
plot(time, Joint5, '-'); 
plot(time, Joint6, '-');

title('Simulated Graph'); 
xlabel('Time (s)'); 
ylabel('Joints (m)'); 

legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'}, 'Location', 'best'); 

grid on; 
hold off;
