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
handles.timer = timer('StartDelay', 2, 'ExecutionMode', 'fixedRate', 'Period', 0.25,... 
    'TimerFcn', {@update_display, hObject});

% This is a previous test
%handles.t = timer( 'Period', 1,'ExecutionMode', 'fixedRate');
%handles.t.TimerFcn = @(x,y)update_display(hObject, eventdata, handles);

% ================= Code adapted from Pol, 2015 =================
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
  
        tmp = fgetl(s); % Read text data
        %fprintf(tmp);
        %[theta error pwm_val gain_num gain_den]
        %Sometimes data is corrupted, so do try method
        try
            [mode force hall opticalR opticalB opticalC opticalG temperature ultrasonic dc_location dc_speed dc_pwm dc_setpoint] = strread(tmp,'%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d');
        catch
            continue;
        end
        % Print values for debugging:
        %fprintf('force: %d\t hall: %d\t opticalR: %d\t opticalB: %d\t opticalC: %d\t opticalG: %d\t temperature: %d\t ultrasonic: %d\n', force, hall, opticalR, opticalB, opticalC, opticalG, temperature, ultrasonic);
        %disp(tmp)

        % Add new values to the end of the current value lists
        handles.force_reading = horzcat(handles.force_reading, force);
        handles.hall_reading = horzcat(handles.hall_reading, hall);
        handles.pot_reading = horzcat(handles.pot_reading,);    %Todo get value from input string
        handles.motor_command_reading = horzcat(handles.motor_command_reading,);
        handles.motor_actual_reading = horzcat(handles.motor_actual_reading,);
        
        %Plot our new values
        axes(handles.force_plot)
        plot(handles.force_reading, 'parent', handles.force_plot);
        axes(handles.hall_plot)
        plot(handles.hall_reading, 'parent', handles.hall_plot);
        axes(handles.potentiometer_plot)
        plot(handles.pot_reading, 'parent', handles.potentiometer_plot);
        hold all
        plot(handles.motor_command_reading, 'r', 'parent', handles.motor_plot);
        plot(handles.motor_actual_reading, 'g', 'parent', handles.motor_plot);
        hold off

        % Display values as floats
        handles.force_string = sprintf('Force Sensor Reading [N]:\n\t      %.2f',force);
        set(handles.force_text, 'String', handles.force_string); 

        handles.hall_string = sprintf('Hall Effect Sensor Reading [micrometers]:\n\t             %.0f',hall);
        set(handles.hall_text, 'String', handles.hall_string); 
        
        handles.pot_string = sprintf('Potentiometer Reading [V]:\n\t             %.0f',potentiometer);
        set(handles.potentiometer_text, 'String', handles.pot_string); 
         
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
    fclose(s)               %Attempt to close the serial connection
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
    user_command_double = str2double(get(handles.motor_edit,'String'));
    %Check if user value is actually a 16-bit signed integer
    if((rem(user_command_double,1)==0) && (abs(user_command_double) < 32768))
        s = handles.s;  %Handle to serial port
        [lsb,msb] = typecast(int16(user_command_double),'int8');
        fwrite(s,[msb,lsb,'\n']);
    else
        set(handles.motor_edit,'String','0')
    end
        
