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

% Last Modified by GUIDE v2.5 03-Feb-2016 08:18:14

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
function mechatronics_gui2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to mechatronics_gui2 (see VARARGIN)

delete(instrfindall) % This deletes previous serial connections

data = guidata(hObject);

% Previously, we set up the lists in these handles.
% Now these just serve to set up our lists
% They will be concatinated to later
handles.force_reading = [0 0];
handles.hall_reading = [0 0];
handles.opticalR_reading = [0 0];
handles.opticalB_reading = [0.001 0.001];
handles.opticalC_reading = [0.002 0.002];
handles.opticalG_reading = [-0.001 -0.001];
handles.temperature_reading = [0 0];
handles.ultrasonic_reading = [0 0];

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
Ts = 0.1;   %PIC sends data every Ts seconds

% Set up serial connection
serial_port_name = '/dev/cu.usbmodem1421'; % CHANGE THIS VALUE TO REFLECT BOARD
s = serial(serial_port_name);
% set specific parameters
s.Baudrate = 9600;
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

% Plot initial values
axes(handles.force_plot)
plot(handles.force_reading);
axes(handles.hall_plot)
plot(handles.hall_reading);
axes(handles.optical_plot)
hold all % so that optical plots can be put on top of each other
plot(handles.opticalR_reading, 'r');
plot(handles.opticalB_reading, 'b');
plot(handles.opticalC_reading, 'k');
plot(handles.opticalG_reading, 'g');
hold off
axes(handles.temperature_plot)
plot(handles.temperature_reading);
axes(handles.ultrasonic_plot)
plot(handles.ultrasonic_reading);

% Choose default command line output for mechatronics_gui2
handles.output = hObject;
handles.s = s; % save the serial identification

% Update handles structure
guidata(hObject, handles);

if strcmp(get(handles.timer, 'Running'), 'off')
    start(handles.timer);
end




% UIWAIT makes mechatronics_gui2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function update_display(hObject, eventdata, hfigure)
%fprintf('update');

handles = guidata(hfigure); %rename this stand-in variable

%data = guidata(hfigure)

s = handles.s;

    while(s.BytesAvailable > 0)
  
        tmp = fgetl(s); % Read text data
        %[theta error pwm_val gain_num gain_den]
        [force hall opticalR opticalB opticalC opticalG temperature ultrasonic] = strread(tmp,'%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d');
        
        % Print values for debugging:
        %fprintf('force: %d\t hall: %d\t opticalR: %d\t opticalB: %d\t opticalC: %d\t opticalG: %d\t temperature: %d\t ultrasonic: %d\n', force, hall, opticalR, opticalB, opticalC, opticalG, temperature, ultrasonic);
        %disp(tmp)

        % Add new values to the end of the current value lists
        handles.force_reading = horzcat(handles.force_reading, force);
        handles.hall_reading = horzcat(handles.hall_reading, hall);
        handles.opticalR_reading = horzcat(handles.opticalR_reading, opticalR);
        handles.opticalB_reading = horzcat(handles.opticalB_reading, opticalB);
        handles.opticalC_reading = horzcat(handles.opticalC_reading, opticalC);
        handles.opticalG_reading = horzcat(handles.opticalG_reading, opticalG);
        handles.temperature_reading = horzcat(handles.temperature_reading, temperature);
        handles.ultrasonic_reading = horzcat(handles.ultrasonic_reading, ultrasonic);

        % Primarily unused x values
        %handles.time = [0: .1: length(handles.ultrasonic_reading];
        
        %Plot our new values)
        axes(handles.force_plot)
        plot(handles.force_reading, 'parent', handles.force_plot);
        axes(handles.hall_plot)
        plot(handles.hall_reading, 'parent', handles.hall_plot);
        axes(handles.optical_plot)
        hold all
        plot(handles.opticalR_reading, 'r', 'parent', handles.optical_plot);
        plot(handles.opticalB_reading, 'b', 'parent', handles.optical_plot);
        plot(handles.opticalC_reading, 'k', 'parent', handles.optical_plot);
        plot(handles.opticalG_reading, 'g', 'parent', handles.optical_plot);
        hold off
        axes(handles.temperature_plot)
        plot(handles.temperature_reading, 'parent', handles.temperature_plot);
        axes(handles.ultrasonic_plot)
        plot(handles.ultrasonic_reading, 'parent', handles.ultrasonic_plot);

        % Display values as floats
        handles.force_string = sprintf('Force Sensor Reading [N]:\n\t      %.2f',force);
        set(handles.force_text, 'String', handles.force_string); 

        handles.hall_string = sprintf('Hall Effect Sensor Reading [micrometers]:\n\t             %.0f',hall);
        set(handles.hall_text, 'String', handles.hall_string); 

        handles.optical_string = sprintf('Optical Sensor Readings [R, B, C, G]:\n%.0f,     %.0f,     %.0f,     %.0f',opticalR, opticalB, opticalC, opticalG);
        set(handles.optical_text, 'String', handles.optical_string); 
        
        handles.temperature_string = sprintf('Temperature Sensor Reading [deg F]:\n\t         %.2f',temperature);
        set(handles.temperature_text, 'String', handles.temperature_string); 

        handles.ultrasonic_string = sprintf('Ultrasonic Sensor Reading [cm]:\n\t         %.2f',ultrasonic);
        set(handles.ultrasonic_text, 'String', handles.ultrasonic_string);  
        fprintf('done')
        % Update figure
        guidata(hfigure, handles);
        

    end
    
% End serial connection if exit  
function mechatronics_gui_CloseRequestFcn(hObject, eventdata, handles)
fprintf('in close');
    if(strcmp(get(handles.timer, 'Running'),'on'))
        stop(handles.timer);
    end
    delete(handles.timer)
    s = handles.s;
    fclose(s)
    delete(hObject);
    delete(instrfindall)



% --- Outputs from this function are returned to the command line.
function varargout = mechatronics_gui2_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function ultrasonic_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ultrasonic_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


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



function servo_edit_Callback(hObject, eventdata, handles)
% hObject    handle to servo_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of servo_edit as text
%        str2double(get(hObject,'String')) returns contents of servo_edit as a double


% --- Executes during object creation, after setting all properties.
function servo_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to servo_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function stepper_edit_Callback(hObject, eventdata, handles)
% hObject    handle to stepper_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of stepper_edit as text
%        str2double(get(hObject,'String')) returns contents of stepper_edit as a double


% --- Executes during object creation, after setting all properties.
function stepper_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to stepper_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function dc_edit_Callback(hObject, eventdata, handles)
% hObject    handle to dc_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of dc_edit as text
%        str2double(get(hObject,'String')) returns contents of dc_edit as a double


% --- Executes during object creation, after setting all properties.
function dc_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dc_edit (see GCBO)
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


% --- Executes on button press in start_pushbutton.
function start_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to start_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%loop(hObject, eventdata, handles)
if strcmp(get(handles.timer, 'Running'), 'off')
    start(handles.timer);
end



% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over start_pushbutton.
function start_pushbutton_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to start_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
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