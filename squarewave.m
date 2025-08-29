% % Parameters
% fs = 6.144e6;              % Sample rate (within Pluto limits)
% f = 100e3;                 % Fundamental frequency (e.g., 100 kHz)
% samplesPerFrame = 5000;
% t = (0:samplesPerFrame-1)' / fs;
% 
% % Square wave with 3 harmonics (band-limited)
% square_wave = sin(2*pi*f*t) + (1/3)*sin(2*pi*3*f*t) + (1/5)*sin(2*pi*5*f*t);
% figure(2);
% plot (t, square_wave);
% % Normalize and make complex
% square_wave = square_wave / max(abs(square_wave));
% txWaveform = single(square_wave + 1j*square_wave);  % Complex signal
% 
% figure(1);
% plot(t,txWaveform,'k')
% xlabel('waveform');
% ylabel('time');
% % figure(2);
% % plot (t, square_wave);

%%
%the correct code
fs = 6.144e6;                  % Baseband sample rate
samplesPerFrame = 5000;       % Number of samples per frame
t = (0:samplesPerFrame-1)' / fs;  % Time vector

% Receiver setup
rx = sdrrx('Pluto', ...
    'RadioID', 'usb:1', ...          % Use 'usb:1' for second device
    'CenterFrequency', 100e6, ...
    'BasebandSampleRate', fs, ...
    'SamplesPerFrame', samplesPerFrame, ...
    'GainSource', 'Manual', ...
    'Gain', 30);

% Real-time plotting loop
for i = 1:100
    rxData = rx();  % Receive samples
    
    figure(1);
    plot(t * 1e3 + (i-1)*samplesPerFrame/fs*1e3, real(rxData));  % ms axis
    title('Received Square Wave');
    xlabel('Time (ms)');
    ylabel('Amplitude');
    grid on;
    drawnow;
end

% Release the SDR
release(rx);

%% WIRELESS TRANSMISSION
%% transmitter side 
%% FOR TRANSMITTING SIGNAL TO ANOTHER SDR CONNECTED TO MUSEERA'S COMPUTER
%Transmitter Side =
fs = 6.144e6;
samplesPerFrame = 5000;
txCenterFreq = 100e6;
txGain = 0;

% Generate 1 MHz sine wave
f = 1e6;
t = (0:samplesPerFrame-1)' / fs;
squareWave = sign(sin(2 * pi * f * t));        % Real square wave: ±1
txWaveform = squareWave + 1j * squareWave;     % Make it complex
txWaveform = single(txWaveform(:)); 
tx = sdrtx('Pluto', ...
    'RadioID', 'usb:0', ...
    'CenterFrequency', txCenterFreq, ...
    'BasebandSampleRate', fs, ...
    'Gain', txGain);

transmitRepeat(tx,txwaveform);
disp('transmitting');












