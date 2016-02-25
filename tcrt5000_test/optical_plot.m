clear all
a = arduino
configurePin(a,'A2','AnalogInput');%Bottom
configurePin(a,'A3','AnalogInput');
configurePin(a,'A4','AnalogInput');
configurePin(a,'A5','AnalogInput');%Top


%Setup plots
i = 2;
%data = [0,0,0,0];
data = zeros(100000,4);
%figure(1);
disp('Start Calibration');
pause;
%Get initial
d1_0 = readVoltage(a,'A2');
d2_0 = readVoltage(a,'A3');
d3_0 = readVoltage(a,'A4');
d4_0 = readVoltage(a,'A5');
pause;

disp('Setup Complete')

while(1)
    d1 = readVoltage(a,'A2')-d1_0;
    d2 = readVoltage(a,'A3')-d2_0;
    d3 = readVoltage(a,'A4')-d3_0;
    d4 = readVoltage(a,'A5')-d4_0;
    data(i,:) = [d1, d2, d3, d4];
    %Find highest differences, average
    cum = 0;
    n_readings = 0;
    if(abs(d1) > 0.1)
        cum = cum + d1;
        n_readings = n_readings + 1;
    end
    if(abs(d2) > 0.1)
        cum = cum + d2;
        n_readings = n_readings + 1;
    end
    if(abs(d3) > 0.1)
        cum = cum + d3;
        n_readings = n_readings + 1;
    end
    if(abs(d4) > 0.1)
        cum = cum + d4;
        n_readings = n_readings + 1;
    end
    avg_out = cum/n_readings;
    [round(d1,2), round(d2,2), round(d3,2), round(d4,2),round(avg_out,2)]
%     figure(1)
%     hold on
%     plot(data(2:i,1),'r');
%     plot(data(2:i,2),'b');
%     plot(data(2:i,3),'g');
%     plot(data(2:i,4));
%     hold off
%     drawnow
    i = i + 1;
    pause;
end

