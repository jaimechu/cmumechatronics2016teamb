% Data organization by column:
% Item code: 0 = no item, refer to spreadsheet for others
% Orientation: 0 = side, 1 = top
% Type: 0 = no item, 1 = plastic, 2 = glass, 3 = metal, 4 = other
% Transparent: 0 = no, 1 = translucent, 2 = transparent
% Red transmissive
% Red reflective
% Green transmissive
% Green reflective
% Blue transmissive
% Blue reflective
% Clear transmissive
% Clear reflective

load('visible_data.mat');
close all;
%Plot individual color: Red
figure;
hold on;
scatter(data(find(data(:,4)==0),5),data(find(data(:,4)==0),6));
scatter(data(find(data(:,4)==1),5),data(find(data(:,4)==1),6));
scatter(data(find(data(:,4)==2),5),data(find(data(:,4)==2),6));
xlabel('Red transmissive frequency (kHz)');
ylabel('Red reflective frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Red');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),7),data(find(data(:,4)==0),8));
scatter(data(find(data(:,4)==1),7),data(find(data(:,4)==1),8));
scatter(data(find(data(:,4)==2),7),data(find(data(:,4)==2),8));
xlabel('Green transmissive frequency (kHz)');
ylabel('Green reflective frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Green');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),9),data(find(data(:,4)==0),10));
scatter(data(find(data(:,4)==1),9),data(find(data(:,4)==1),10));
scatter(data(find(data(:,4)==2),9),data(find(data(:,4)==2),10));
xlabel('Blue transmissive frequency (kHz)');
ylabel('Blue reflective frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Blue');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),11),data(find(data(:,4)==0),12));
scatter(data(find(data(:,4)==1),11),data(find(data(:,4)==1),12));
scatter(data(find(data(:,4)==2),11),data(find(data(:,4)==2),12));
xlabel('Clear transmissive frequency (kHz)');
ylabel('Clear reflective frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Clear');
hold off;

%Plot different colors
figure;
hold on;
scatter(data(find(data(:,4)==0),5),data(find(data(:,4)==0),7));
scatter(data(find(data(:,4)==1),5),data(find(data(:,4)==1),7));
scatter(data(find(data(:,4)==2),5),data(find(data(:,4)==2),7));
xlabel('Red transmissive frequency (kHz)');
ylabel('Green transmissive frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Red vs. Green');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),5),data(find(data(:,4)==0),8));
scatter(data(find(data(:,4)==1),5),data(find(data(:,4)==1),8));
scatter(data(find(data(:,4)==2),5),data(find(data(:,4)==2),8));
xlabel('Red transmissive frequency (kHz)');
ylabel('Green reflective frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Red vs. Green');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),5),data(find(data(:,4)==0),9));
scatter(data(find(data(:,4)==1),5),data(find(data(:,4)==1),9));
scatter(data(find(data(:,4)==2),5),data(find(data(:,4)==2),9));
xlabel('Red transmissive frequency (kHz)');
ylabel('Blue transmissive frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Red vs. Blue');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),5),data(find(data(:,4)==0),10));
scatter(data(find(data(:,4)==1),5),data(find(data(:,4)==1),10));
scatter(data(find(data(:,4)==2),5),data(find(data(:,4)==2),10));
xlabel('Red transmissive frequency (kHz)');
ylabel('Blue reflective frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Red vs. Blue');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),5),data(find(data(:,4)==0),11));
scatter(data(find(data(:,4)==1),5),data(find(data(:,4)==1),11));
scatter(data(find(data(:,4)==2),5),data(find(data(:,4)==2),11));
xlabel('Red transmissive frequency (kHz)');
ylabel('Clear transmissive frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Red vs. Clear');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),5),data(find(data(:,4)==0),12));
scatter(data(find(data(:,4)==1),5),data(find(data(:,4)==1),12));
scatter(data(find(data(:,4)==2),5),data(find(data(:,4)==2),12));
xlabel('Red transmissive frequency (kHz)');
ylabel('Clear reflective frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Red vs. Green');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),6),data(find(data(:,4)==0),7));
scatter(data(find(data(:,4)==1),6),data(find(data(:,4)==1),7));
scatter(data(find(data(:,4)==2),6),data(find(data(:,4)==2),7));
xlabel('Red reflective frequency (kHz)');
ylabel('Green transmissive frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Red vs. Green');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),6),data(find(data(:,4)==0),8));
scatter(data(find(data(:,4)==1),6),data(find(data(:,4)==1),8));
scatter(data(find(data(:,4)==2),6),data(find(data(:,4)==2),8));
xlabel('Red reflective frequency (kHz)');
ylabel('Green reflective frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Red vs. Green');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),6),data(find(data(:,4)==0),9));
scatter(data(find(data(:,4)==1),6),data(find(data(:,4)==1),9));
scatter(data(find(data(:,4)==2),6),data(find(data(:,4)==2),9));
xlabel('Red reflective frequency (kHz)');
ylabel('Blue transmissive frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Red vs. Blue');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),6),data(find(data(:,4)==0),10));
scatter(data(find(data(:,4)==1),6),data(find(data(:,4)==1),10));
scatter(data(find(data(:,4)==2),6),data(find(data(:,4)==2),10));
xlabel('Red reflective frequency (kHz)');
ylabel('Blue reflective frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Red vs. Blue');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),6),data(find(data(:,4)==0),11));
scatter(data(find(data(:,4)==1),6),data(find(data(:,4)==1),11));
scatter(data(find(data(:,4)==2),6),data(find(data(:,4)==2),11));
xlabel('Red reflective frequency (kHz)');
ylabel('Clear transmissive frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Red vs. Clear');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),6),data(find(data(:,4)==0),12));
scatter(data(find(data(:,4)==1),6),data(find(data(:,4)==1),12));
scatter(data(find(data(:,4)==2),6),data(find(data(:,4)==2),12));
xlabel('Red reflective frequency (kHz)');
ylabel('Clear reflective frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Red vs. Clear');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),7),data(find(data(:,4)==0),9));
scatter(data(find(data(:,4)==1),7),data(find(data(:,4)==1),9));
scatter(data(find(data(:,4)==2),7),data(find(data(:,4)==2),9));
scatter(data(find(data(:,3)==3),7),data(find(data(:,3)==3),9));
xlabel('Green transmissive frequency (kHz)');
ylabel('Blue transmissive frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Green vs. Blue');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),7),data(find(data(:,4)==0),10));
scatter(data(find(data(:,4)==1),7),data(find(data(:,4)==1),10));
scatter(data(find(data(:,4)==2),7),data(find(data(:,4)==2),10));
xlabel('Green transmissive frequency (kHz)');
ylabel('Blue reflective frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Green vs. Blue');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),7),data(find(data(:,4)==0),11));
scatter(data(find(data(:,4)==1),7),data(find(data(:,4)==1),11));
scatter(data(find(data(:,4)==2),7),data(find(data(:,4)==2),11));
xlabel('Green transmissive frequency (kHz)');
ylabel('Clear transmissive frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Green vs. Clear');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),7),data(find(data(:,4)==0),12));
scatter(data(find(data(:,4)==1),7),data(find(data(:,4)==1),12));
scatter(data(find(data(:,4)==2),7),data(find(data(:,4)==2),12));
xlabel('Green transmissive frequency (kHz)');
ylabel('Clear reflective frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Green vs. Clear');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),8),data(find(data(:,4)==0),9));
scatter(data(find(data(:,4)==1),8),data(find(data(:,4)==1),9));
scatter(data(find(data(:,4)==2),8),data(find(data(:,4)==2),9));
xlabel('Green reflective frequency (kHz)');
ylabel('Blue transmissive frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Green vs. Blue');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),8),data(find(data(:,4)==0),10));
scatter(data(find(data(:,4)==1),8),data(find(data(:,4)==1),10));
scatter(data(find(data(:,4)==2),8),data(find(data(:,4)==2),10));
xlabel('Green reflective frequency (kHz)');
ylabel('Blue reflective frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Green vs. Blue');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),8),data(find(data(:,4)==0),11));
scatter(data(find(data(:,4)==1),8),data(find(data(:,4)==1),11));
scatter(data(find(data(:,4)==2),8),data(find(data(:,4)==2),11));
xlabel('Green reflective frequency (kHz)');
ylabel('Clear transmissive frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Green vs. Clear');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),8),data(find(data(:,4)==0),12));
scatter(data(find(data(:,4)==1),8),data(find(data(:,4)==1),12));
scatter(data(find(data(:,4)==2),8),data(find(data(:,4)==2),12));
xlabel('Green reflective frequency (kHz)');
ylabel('Clear reflective frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Green vs. Clear');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),9),data(find(data(:,4)==0),11));
scatter(data(find(data(:,4)==1),9),data(find(data(:,4)==1),11));
scatter(data(find(data(:,4)==2),9),data(find(data(:,4)==2),11));
xlabel('Blue transmissive frequency (kHz)');
ylabel('Clear transmissive frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Blue vs. Clear');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),9),data(find(data(:,4)==0),12));
scatter(data(find(data(:,4)==1),9),data(find(data(:,4)==1),12));
scatter(data(find(data(:,4)==2),9),data(find(data(:,4)==2),12));
xlabel('Blue transmissive frequency (kHz)');
ylabel('Clear reflective frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Blue vs. Clear');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),10),data(find(data(:,4)==0),11));
scatter(data(find(data(:,4)==1),10),data(find(data(:,4)==1),11));
scatter(data(find(data(:,4)==2),10),data(find(data(:,4)==2),11));
xlabel('Blue reflective frequency (kHz)');
ylabel('Clear transmissive frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Blue vs. Clear');
hold off;

figure;
hold on;
scatter(data(find(data(:,4)==0),10),data(find(data(:,4)==0),12));
scatter(data(find(data(:,4)==1),10),data(find(data(:,4)==1),12));
scatter(data(find(data(:,4)==2),10),data(find(data(:,4)==2),12));
xlabel('Blue reflective frequency (kHz)');
ylabel('Clear reflective frequency (kHz)');
legend('Opaque','Translucent','Transparent');
title('TCS3200 Transparency test: Blue vs. Clear');
hold off;