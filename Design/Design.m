import PressureData.txt;
A =textscan('PressureData.txt','%f%f');

x = A(:,1);
y = A(:,2);
scatter(x,y);
xlabel('Time');
ylabel('Pressure')
figure; plot(x,y,'+')
xvalue = [0:0.05:5];
%p = polyfit(A,B,3);
Y = polyval(p,xvalues);
hold on;
plot(xvalue,Y);
% displacement = integral of velocity 

import AccelerationData.txt;
Acc = textscan('AccelerationData.txt','%f%f');

time = Acc(:,1);
Acceleration = Acc(:,2);
timeinterval = 0.05;
speed = timeinterval .* Acceleration;
distance = sum(abs(speed)) * timeinterval;





