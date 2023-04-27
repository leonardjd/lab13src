% Calculate Distance to Black Line in Image
%      X1      
pts = [472,400,370,339,306]'         
ipts = (1)./pts;
i2pts = ipts.*ipts;
v1 = [ones(size(pts,1),1)];
%Distance in inches
D = [8;12;15;20.5;30];
dP =  [ ipts,v1];   %Linear
dPP = [i2pts, ipts,v1]; %Quad

dP
dPP
par1 = dP\D;                %slope and y intercept
par2 = dPP\D;               %quadratic, slope, and y intercept
m = par1(1);
b = par1(2);
fprintf(1,'Linear m %5.2f,  y intercept %5.2f\n',m,b);
Dcalc = m*dP(:,1) + b*ones(size(D))
%Plot Dcalc vs dP and D vs dP
figure(1);
plot(Dcalc, D);
hold on;
plot(D,D);
hold off;
title('Robot #0 Linear');
xlabel('Dcalc');
ylabel('Distance (inches)');
legend('Estimated','Measured','Location','Southeast');

a = par2(1);            %quadratic
m2 = par2(2);           %slope
b2 = par2(3);           %y intercept
fprintf(1,'Quadratic a %5.3f  m %5.2f,  y intercept %5.2f\n',a,m2,b2);
D2calc = a*dPP(:,1) + m2*dPP(:,2) + b2*ones(size(D))

figure(3);
plot(D,D);
hold on;
plot(D2calc,D);
hold off;
title('Robot #0 Quadratic');
xlabel('Calculated Distance');
ylabel('Distance (inches)');
legend('Estimated','Measured','Location','Southeast');