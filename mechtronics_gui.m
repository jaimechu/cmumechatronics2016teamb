function varargout = mechatronics_gui(varargin)
% MECHATRONICS_GUI MATLAB code for mechatronics_gui.fig
%      MECHATRONICS_GUI, by itself, creates a new MECHATRONICS_GUI or raises the existing
%      singleton*.
%
%      H = MECHATRONICS_GUI returns the handle to a new MECHATRONICS_GUI or the handle to
%      the existing singleton*.
%
%      MECHATRONICS_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MECHATRONICS_GUI.M with the given input arguments.
%
%      MECHATRONICS_GUI('Property','Value',...) creates a new MECHATRONICS_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before mechatronics_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to mechatronics_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help mechatronics_gui

% Last Modified by GUIDE v2.5 01-Feb-2016 18:52:54

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @mechatronics_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @mechatronics_gui_OutputFcn, ...
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

% --- Executes just before mechatronics_gui is made visible.
function mechatronics_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to mechatronics_gui (see VARARGIN)

handles.force_reading = [3,3,3,3,1,3];
handles.inductance_reading = [2,0,2,8,2,2];
handles.optical_reading = [4,4,4,8,4,4];
handles.temperature_reading = [10,20,30,40,50,20];
handles.ultrasonic_reading = [1,9,1,9,1,1];

handles.time = [1,2,3,4,5,6];

axes(handles.force_plot)
plot(handles.time, handles.force_reading);
axes(handles.inductance_plot)
plot(handles.time, handles.inductance_reading);
axes(handles.optical_plot)
plot(handles.time, handles.optical_reading);
axes(handles.temperature_plot)
plot(handles.time, handles.temperature_reading);
axes(handles.ultrasonic_plot)
plot(handles.time, handles.ultrasonic_reading);

% Choose default command line output for mechatronics_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes mechatronics_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = mechatronics_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function loop()
fprintf('hi')
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
for i = 1:1000
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
    if(s.BytesAvailable > 0)
  
        tmp = fgetl(s); %Read text data
        %[theta error pwm_val gain_num gain_den]
        [A B C] = strread(tmp,'%d\t%d\t%d');
        fprintf('A: %d\t B: %d\t C: %d\t\n', A, B, C);
        %disp(tmp)
        handles.force_reading = [3,3,3,3,1,3];
        handles.inductance_reading = [2,0,2,8,2,2];
        handles.optical_reading = [4,4,4,8,4,4];
        handles.temperature_reading = [10,20,30,40,50,20];
        handles.ultrasonic_reading = [1,9,1,9,1,1];

        handles.time = [1,2,3,4,5,6];

        axes(handles.force_plot)
        plot(handles.time, handles.force_reading);
        axes(handles.inductance_plot)
        plot(handles.time, handles.inductance_reading);
        axes(handles.optical_plot)
        plot(handles.time, handles.optical_reading);
        axes(handles.temperature_plot)
        plot(handles.time, handles.temperature_reading);
        axes(handles.ultrasonic_plot)
        plot(handles.time, handles.ultrasonic_reading);
        
        guidata(hObject, handles);
        
        i=i+1;
        
    end
end
fclose(s);

% --- Executes during object creation, after setting all properties.
function text2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text2 (see GCBO)
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



function sevo_edit_Callback(hObject, eventdata, handles)
% hObject    handle to sevo_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of sevo_edit as text
%        str2double(get(hObject,'String')) returns contents of sevo_edit as a double


% --- Executes during object creation, after setting all properties.
function sevo_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sevo_edit (see GCBO)
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
loop()


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
function inductance_plot_CreateFcn(hObject, eventdata, handles)
% hObject    handle to inductance_plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate inductance_plot
