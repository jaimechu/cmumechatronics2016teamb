function varargout = mechatronics_gui2(varargin)
% delete(instrfindall)
% MECHATRONICS_GUI2 MATLAB code for mechatronics_gui2.fig
%      MECHATRONICS_GUI2, by itself, creates a new MECHATRONICS_GUI2 or raises the existing
%      singleton*.
%
%      H = MECHATRONICS_GUI2 returns the handle to a new MECHATRONICS_GUI2 or the handle to
%      the existing singleton*.
%
%      MECHATRONICS_GUI2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MECHATRONICS_GUI2.M with the given input arguments.
%
%      MECHATRONICS_GUI2('Property','Value',...) creates a new MECHATRONICS_GUI2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before mechatronics_gui2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to mechatronics_gui2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help mechatronics_gui2

% Last Modified by GUIDE v2.5 07-Feb-2016 19:44:38

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @mechatronics_gui2_OpeningFcn, ...
                   'gui_OutputFcn',  @mechatronics_gui2_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before mechatronics_gui2 is made visible.
function mechatronics_gui2_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to mechatronics_gui2 (see VARARGIN)

delete(instrfindall) % This deletes previous serial connections

%data = guidata(hObject);

% Previously, we set up the lists in these handles.
% Now these just serve to set up our lists
% They will be concatinated to later
handles.force_reading = [0 0];
handles.hall_reading = [0 0];
handles.pot_reading = [0 0];
% Lists are shared for motors and are reset every time mode is switched
% One list for commanded value done through GUI or sensor
handles.motor_command_reading = [0 0];
% One list for actual value
% For dc motor, actual value is encoder reading
% For stepper, this is open-loop position
% For servo, copy of commanded value (Arduino sends back commanded value)
handles.motor_actual_reading = [0 0];

% Now we set up a timer. 
% It will wait 2 seconds.
% Every 0.1 seconds, it will call update_display
handles.timer = timer('StartDelay', 2, 'ExecutionMode', 'fixedRate', 'Period', 0.1,... 
    'TimerFcn', {@update_display, hObject});

% This is a previous test
%handles.t = timer( 'Period', 1,'ExecutionMode', 'fixedRate');
%handles.t.TimerFcn = @(x,y)update_display(hObject, eventdata, handles);

% ================= Arduino interface Code adapted from Pol, 2015 (18-474 final lab)=================
% Create array to hold values from Arduino
num_rows = 1000;
num_cols = 8;
%Ts = 0.1;   %PIC sends data every Ts seconds

% Set up serial connection
serial_port_name = 'COM19';%'/dev/cu.usbmodem1421'; % CHANGE THIS VALUE TO REFLECT BOARD AND PC
s = serial(serial_port_name);
% set specific parameters
s.Baudrate = 115200;
s.DataBits = 8;
s.FlowControl = 'none';
s.Parity = 'none';
s.StopBits = 1;
s.InputBufferSize = num_rows*num_cols*2;    % 16 bit data 2 cols
disp('Serial port created. Press Enter to open the serial port');
pause() % wait for enter

fopen(s); % open file for reading and writing
% wait data for data in the serial port buffer    
 
% Primarily unused x values:    
handles.time = [];

% Plot initial dummy values
axes(handles.force_plot)
plot(handles.force_reading);
axes(handles.hall_plot)
plot(handles.hall_reading);
axes(handles.potentiometer_plot)
plot(handles.pot_reading);
hold all;    %Plot both motor command and actual on same plot
axes(handles.motor_plot)
plot(handles.motor_command_reading,'r');
plot(handles.motor_actual_reading,'g');
hold off;

% Choose default command line output for mechatronics_gui2
handles.output = hObject;
handles.s = s; % save the serial identification
handles.mode_prev = 0;  %Previously reported mode, to check for mode changes
handles.op_mode = 0;

% Update handles structure
guidata(hObject, handles);

%Start timer to update GUI
if strcmp(get(handles.timer, 'Running'), 'off')
    start(handles.timer);
end




% UIWAIT makes mechatronics_gui2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% This function is the update loop that checks for data from the Arduino,
% and accordingly updates the plots
function update_display(hObject, eventdata, hfigure)
%fprintf('update\n');
%handles is not passed in, get it from hfigure, which is passed in
handles = guidata(hfigure); %rename this stand-in variable

s = handles.s;

    while(s.BytesAvailable > 0)%While data is ready to be read
        % Plotting is slow, so the serial buffer gets filled
        % and slows down the gui, so throw out some data
        for n=1:100
            fgetl(s);
        end
        tmp = fgetl(s) % Read text data
        %fprintf(tmp);
        %[theta error pwm_val gain_num gain_den]
        %Sometimes data is corrupted, so do try method
        try
            [mode, force, hall, pot, dc_location, dc_speed, dc_cmd, dc_setpoint, servo, step_setpoint, step_current, gui_read] = strread(tmp,'%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d');
        catch
            continue;
        end
        % Print values for debugging:
        %fprintf('force: %d\t hall: %d\t opticalR: %d\t opticalB: %d\t opticalC: %d\t opticalG: %d\t temperature: %d\t ultrasonic: %d\n', force, hall, opticalR, opticalB, opticalC, opticalG, temperature, ultrasonic);
        %disp(tmp)
        
        %Change raw readings into actual units
        pot = pot*5/1024;
        
        %If mode has changed, motor lists need to be cleared
        if(mode ~= handles.mode_prev)
            handles.motor_command_reading = [0 0];
            handles.motor_actual_reading = [0 0];
        end
        handles.mode_prev = mode;

        % Add new sensor values to the end of the current value lists
        handles.force_reading = horzcat(handles.force_reading, force);
        handles.hall_reading = horzcat(handles.hall_reading, hall);
        handles.pot_reading = horzcat(handles.pot_reading,pot);    %Todo get value from input string
        
        % Add new motor values to the end of the current value lists
        % Because lists are shared for all motors, the values added depend
        % on the motor
        switch(mode)
            case 0 %MODE_DC_POT_SPEED
                handles.motor_command_reading = horzcat(handles.motor_command_reading,dc_setpoint);
                handles.motor_actual_reading = horzcat(handles.motor_actual_reading,dc_speed);
            case 1 %MODE_DC_POT_POS
                handles.motor_command_reading = horzcat(handles.motor_command_reading,dc_setpoint);
                handles.motor_actual_reading = horzcat(handles.motor_actual_reading,dc_location);
            case 2 %MODE_STEP_HALL
                handles.motor_command_reading = horzcat(handles.motor_command_reading,step_setpoint);
                handles.motor_actual_reading = horzcat(handles.motor_actual_reading,step_current);
            case 3 %MODE_SERVO_FSR
                handles.motor_command_reading = horzcat(handles.motor_command_reading,servo);
                handles.motor_actual_reading = horzcat(handles.motor_actual_reading,servo);
            case 4 %MODE_DC_GUI_SPEED
                handles.motor_command_reading = horzcat(handles.motor_command_reading,dc_setpoint);
                handles.motor_actual_reading = horzcat(handles.motor_actual_reading,dc_speed);
            case 5 %MODE_DC_GUI_POS
                handles.motor_command_reading = horzcat(handles.motor_command_reading,dc_setpoint);
                handles.motor_actual_reading = horzcat(handles.motor_actual_reading,dc_location);
            case 6 %MODE_STEP_GUI
                handles.motor_command_reading = horzcat(handles.motor_command_reading,step_setpoint);
                handles.motor_actual_reading = horzcat(handles.motor_actual_reading,step_current);
            case 7 %MODE_SERVO_GUI
                handles.motor_command_reading = horzcat(handles.motor_command_reading,servo);
                handles.motor_actual_reading = horzcat(handles.motor_actual_reading,servo);
        end
        
        %Plot our new values
        hold on
        plot(handles.motor_command_reading, 'r', 'parent', handles.motor_plot);
        plot(handles.motor_actual_reading, 'g', 'parent', handles.motor_plot);
        hold off
        
        axes(handles.force_plot)
        plot(handles.force_reading, 'parent', handles.force_plot);
        axes(handles.hall_plot)
        plot(handles.hall_reading, 'parent', handles.hall_plot);
        axes(handles.potentiometer_plot)
        plot(handles.pot_reading, 'parent', handles.potentiometer_plot);


        % Display values as floats
        handles.force_string = sprintf('Force Sensor Reading [N]:\n\t      %.2f',force);
        set(handles.force_text, 'String', handles.force_string); 

        handles.hall_string = sprintf('Hall Effect Sensor Reading [micrometers]:\n\t             %.0f',hall);
        set(handles.hall_text, 'String', handles.hall_string); 
        
        handles.pot_string = sprintf('Potentiometer Reading [V]:\n\t             %.0f',pot);
        set(handles.potentiometer_text, 'String', handles.pot_string); 
        
        %Update mode display and disable/enable command input
        mode_str = '';
        switch(mode)
            case 0 %MODE_DC_POT_SPEED
                mode_str = 'DC_POT_SPEED';
                %GUI does not command motor, disable command input
%                 if(strcmp(get(handles.motor_edit, 'Enable'),'off'))
%                     set(handles.motor_edit, 'Enable', 'off');
%                     set(handles.motor_edit, 'String', '0');
%                     set(handles.sendButton, 'Enable', 'off');
%                 end
            case 1 %MODE_DC_POT_POS
                mode_str = 'DC_POT_POS';
                 %GUI does not command motor, disable command input
%                if(strcmp(get(handles.motor_edit, 'Enable'),'off'))
%                     set(handles.motor_edit, 'Enable', 'off');
%                     set(handles.motor_edit, 'String', '0');
%                     set(handles.sendButton, 'Enable', 'off');
%                 end
            case 2 %MODE_STEP_HALL
                mode_str = 'STEP_HALL';
                %GUI does not command motor, disable command input
%                 if(strcmp(get(handles.motor_edit, 'Enable'),'off'))
%                     set(handles.motor_edit, 'Enable', 'off');
%                     set(handles.motor_edit, 'String', '0');
%                     set(handles.sendButton, 'Enable', 'off');
%                 end
            case 3 %MODE_SERVO_FSR
                mode_str = 'SERVO_FSR';
                %GUI does not command motor, disable command input
%                 if(strcmp(get(handles.motor_edit, 'Enable'),'off'))
%                     set(handles.motor_edit, 'Enable', 'off');
%                     set(handles.motor_edit, 'String', '0');
%                     set(handles.sendButton, 'Enable', 'off');
%                 end
            case 4 %MODE_DC_GUI_SPEED
                mode_str = 'DC_GUI_SPEED';
                %GUI has control of motor, enable command input
%                 if(strcmp(get(handles.motor_edit, 'Enable'),'on'))
%                     set(handles.motor_edit, 'Enable', 'on');
%                     set(handles.motor_edit, 'String', '0');
%                     set(handles.sendButton, 'Enable', 'on');
%                 end
            case 5 %MODE_DC_GUI_POS
                mode_str = 'DC_GUI_POS';
                %GUI has control of motor, enable command input
%                 if(strcmp(get(handles.motor_edit, 'Enable'),'on'))
%                     set(handles.motor_edit, 'Enable', 'on');
%                     set(handles.motor_edit, 'String', '0');
%                     set(handles.sendButton, 'Enable', 'on');
%                 end
            case 6 %MODE_STEP_GUI
                mode_str = 'STEP_GUI';
                %GUI has control of motor, enable command input
%                 if(strcmp(get(handles.motor_edit, 'Enable'),'on'))
%                     set(handles.motor_edit, 'Enable', 'on');
%                     set(handles.motor_edit, 'String', '0');
%                     set(handles.sendButton, 'Enable', 'on');
%                 end
            case 7 %MODE_SERVO_GUI
                mode_str = 'SERVO_GUI';
                %GUI has control of motor, enable command input
%                 if(strcmp(get(handles.motor_edit, 'Enable'),'on'))
%                     set(handles.motor_edit, 'Enable', 'on');
%                     set(handles.motor_edit, 'String', '0');
%                     set(handles.sendButton, 'Enable', 'on');
%                 end
        end
        handles.op_mode = mode;
        handles.mode_text_str = sprintf('System Mode:\n\t%s',mode_str);
        set(handles.modeText,'String',handles.mode_text_str);
         
        %fprintf('done')
        % Update figure
        guidata(hfigure, handles);
        pause(0.01);
        

    end
    
% End serial connection if exit  
function mechatronics_gui_CloseRequestFcn(hObject, eventdata, handles)
fprintf('in close');
    if(strcmp(get(handles.timer, 'Running'),'on'))
        stop(handles.timer);
    end
    delete(handles.timer)   %Delete the timer so callback doesn't execute
    s = handles.s;
    fclose(s);              %Attempt to close the serial connection
    delete(hObject);        %Delete the GUI
    delete(instrfindall)    %Close any lingering serial connections



% --- Outputs from this function are returned to the command line.
function varargout = mechatronics_gui2_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% % --- Executes on selection change in plot_popup.
% function plot_popup_Callback(hObject, eventdata, handles)
% % hObject    handle to plot_popup (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % Hints: contents = cellstr(get(hObject,'String')) returns plot_popup contents as cell array
% %        contents{get(hObject,'Value')} returns selected item from plot_popup
% 
% val = get(hObject, 'Value');
% str = get(hObject, 'String');
% if (strcmp(str(val),'Ultrasonic') == true)
%         handles.current_data = handles.ultrasonic_reading;
% elseif (strcmp(str(val), 'Inductance') == true)
%         handles.current_data = handles.inductance_reading;
% elseif (strcmp(str(val), 'Force') == true)
%         handles.current_data = handles.force_reading;
% elseif (strcmp(str(val), 'Optical') == true)
%         handles.current_data = handles.optical_reading;    
% end
% guidata(hObject, handles);
% plot(handles.time, handles.current_data);



% --- Executes during object creation, after setting all properties.
function plot_popup_CreateFcn(hObject, eventdata, handles)
% hObject    handle to plot_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function motor_edit_Callback(hObject, eventdata, handles)
% hObject    handle to motor_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of motor_edit as text
%        str2double(get(hObject,'String')) returns contents of motor_edit as a double


% --- Executes during object creation, after setting all properties.
function motor_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motor_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on key press with focus on plot_popup and none of its controls.
function plot_popup_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to plot_popup (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)



% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over plot_popup.
function plot_popup_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to plot_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function hall_plot_CreateFcn(hObject, eventdata, handles)
% hObject    handle to hall_plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate hall_plot


% --- Executes during object creation, after setting all properties.
function force_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to force_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function force_plot_CreateFcn(hObject, eventdata, handles)
% hObject    handle to force_plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate force_plot


% --- Executes on mouse press over axes background.
function force_plot_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to force_plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in sendButton.
function sendButton_Callback(hObject, eventdata, handles)
% hObject    handle to sendButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    get(handles.motor_edit,'String')
    user_command_double = str2double(get(handles.motor_edit,'String'))
    if(handles.op_mode == 6)
        %User input in degrees, change to steps
        user_command_double = round(user_command_double);%user_command_double*200/360);%200/360
    elseif(handles.op_mode == 5)
        user_command_double = round(user_command_double*60/360);
    end
    handles.op_mode
    
    user_command_double
    %temp = typecast(int16(user_command_double),'int8')
    lsb = cast(bitand(int16(user_command_double),255),'int8')
    msb = cast(bitsra(int16(user_command_double),8),'int8')
    unicodestr = native2unicode([lsb,msb,int8(0)]);
    unicode2native(unicodestr)
    fwrite(handles.s,unicodestr);
    
%     %Check if user value is actually a 16-bit signed integer
%     if((rem(user_command_double,1)==0) && (abs(user_command_double) < 32768))
%         s = handles.s;  %Handle to serial port
%         % If number is less than 256, typecast doesn't give a value for
%         % msb, so use try catch to take care of small values.
%         try
%             [lsb,msb] = typecast(int16(user_command_double),'int8')
%         catch
%             lsb = typecast(int16(user_command_double),'int8')
%             if(lsb >= 0)
%                 msb = int8(0);
%             else
%                 msb = int8(-128);
%             end
%         end
%         fwrite(s,[msb,lsb,'\n']);
%     else
%         set(handles.motor_edit,'String','0')
%     end
        
