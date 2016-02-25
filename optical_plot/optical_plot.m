clear all
S0 = 'D2';%Output frequency
S1 = 'D3';
S2 = 'D4';%Filter type
S3 = 'D5';
a = arduino
configurePin(a,'A0','AnalogInput');
configurePin(a,S0,'DigitalOutput');
configurePin(a,S1,'DigitalOutput');
configurePin(a,S2,'DigitalOutput');
configurePin(a,S3,'DigitalOutput');

%Frequency scaling
writeDigitalPin(a,S0,1);
writeDigitalPin(a,S1,1);

%Setup plots
i = 2;
%data = [0,0,0,0];
data = zeros(100000,4);
figure(1);

disp('Setup Complete')

while(1)
    writeDigitalPin(a,S2,0);%Red
    writeDigitalPin(a,S3,0);
    %pause(0.1);
    red = readVoltage(a,'A0');
    writeDigitalPin(a,S2,0);%Blue
    writeDigitalPin(a,S3,1);
    %pause(0.1);
    blue = readVoltage(a,'A0');
    writeDigitalPin(a,S2,1);%Clear
    writeDigitalPin(a,S3,0);
    %pause(0.1);
    white = readVoltage(a,'A0');
    writeDigitalPin(a,S2,1);%Green
    writeDigitalPin(a,S3,1);
    %pause(0.1);
    green = readVoltage(a,'A0');
    [red, blue, green, white]
    %data = [data; [red, blue, green, white]];
    data(i,:) = [red, blue, green, white];
    figure(1)
    hold on
    plot(data(2:i,1),'r');
    plot(data(2:i,2),'b');
    plot(data(2:i,3),'g');
    plot(data(2:i,4));
    hold off
    drawnow
    i = i + 1;
end

