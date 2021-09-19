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
    vel(i,1)=IncomingString(1,2);

    angle_value=str2double(angle);
    angle_value = angle_value.';
    vel_value=str2double(vel);
    vel_value = vel_value.';

    i=i+1;

    if(s.Status ~= 'open')
        break;
    end
        
    
end
subplot(2,1,1);
plot(angle_value);
subplot(2,1,2);
plot(vel_value);

%data=flushinput(s);
fclose(s);

instrfind();
fclose(ans);
