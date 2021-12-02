clear
clc
close all
% remove any persistant serial connections
if ~isempty(instrfind('Status', 'open'))
fclose(instrfind);
end
% Create connections to each device
stm_device = serial('COM3', 'BaudRate', 115200, 'Timeout', 0.5, 'Terminator', 'LF');
fopen(stm_device);
% Call the potentiometer data-logging code. 'compose' appends the correct
% new-line characters to the string
txStr = compose("getEncoder start");
fprintf(stm_device, txStr);
% Define expected number of samples
N = 400;
% Initialise data structure for data
encoder_raw = zeros(N,1);
% Collect the expected number of samples
for i=1:N
    % Read in data from serial connection
    rxStr = fgets(stm_device);
    % Convert data to numbers and store in data structure
    encoder_raw(i) = str2double(rxStr);
end


encoder = (encoder_raw/4096)*(2*pi);
save('encoder.mat')
save('N.mat')

%Plot
figure(2);
grid on;
plot(encoder, 'LineWidth',3)
xlabel('Sample [Ts]')
ylabel('Encoder Angle [degrees]')
title('Encoder Measurements')

% Teardown
fclose(stm_device);
delete(stm_device);
clear stm_device % Close serial connection and clean up
clear stm_device % Close serial connection and clean up