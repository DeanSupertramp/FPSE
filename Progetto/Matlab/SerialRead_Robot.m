%%
clc;
close all;
clear all;

instrfind();
fclose(ans);
%%
s = serial('COM9', 'BaudRate', 115200);

fopen(s);
%%
i=1;
x=linspace(1,115200,500);
for i=1:length(x)   
    data = fscanf(s);
    IncomingString = char(data);
    while length(data)<31
        data = fscanf(s);
        IncomingString = char(data);
    end
    IncomingString = regexp(IncomingString, ' ', 'split');
    angle(i,1)=IncomingString(1,1);
    Gy(i,1)=IncomingString(1,2);
    pitch(i,1)=IncomingString(1,3);

    angle_value=str2double(angle);
    angle_value = angle_value.';
    Gy_value=str2double(Gy);
    Gy_value = Gy_value.';
    pitch_value=str2double(pitch);
    pitch_value = pitch_value.';

    i=i+1;

    if(s.Status ~= 'open')
        break;
    end
        
    
end
plot(angle_value);
hold on;
plot(Gy_value);
hold on;
plot(pitch_value);

%data=flushinput(s);
fclose(s);

instrfind();
fclose(ans);
