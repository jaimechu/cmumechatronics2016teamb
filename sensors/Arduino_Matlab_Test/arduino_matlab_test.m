clear all
close all
clc; 
num_rows = 10;
num_cols = 10;
Ts = 0.1;   %PIC sends data every Ts seconds
serial_port_name = 'COM20';
s = serial(serial_port_name);
% set specific parameters
s.Baudrate = 9600;
s.DataBits = 8;
s.FlowControl = 'none';
s.Parity = 'none';
s.StopBits = 1;
s.InputBufferSize = num_rows*num_cols*2;    % 16 bit data 2 cols
disp('Serial port created. Press Enter to open the serial port');
pause; % waiting

fopen(s); % open file for reading and writing
% wait data for data in the serial port buffer
pause(1);
no_data = 0;
disp('Transferring data ...');
counter = 0;
%for i = 1:length(k)
while 1==1
    %Output reference signal
    %toSend = int16(ref(i));
    %firstByte = uint8(bitand(toSend,255));
    %secondByte = uint8(bitand(bitshift(toSend,-8),255));
    
    %fwrite(s, firstByte);
    %fwrite(s, secondByte);
    
    t(counter+1) = counter * Ts;
    counter = counter + 1;
    
   % pause(0.1);
    timer = 0;
    while(s.BytesAvailable == 0)
    end
    tmp = fgetl(s); %Read text data
    %[theta error pwm_val gain_num gain_den]
    [A B C] = strread(tmp,'%d\t%d\t%d');
    fprintf('A: %d\t B: %d\t C: %d\t\n', A, B, C);
    %disp(tmp)
end
fclose(s);