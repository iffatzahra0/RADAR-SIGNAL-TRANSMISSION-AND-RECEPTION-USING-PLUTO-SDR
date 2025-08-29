%% Doppler Radar using ADALM-PLUTO
% Parameters
fc = 2.4e9;              % Carrier frequency (Hz)
fs = 1e6;                % Baseband sample rate (Hz)
c = 3e8;                 % Speed of light (m/s)
v = 10;                  % Target velocity (m/s)
samplesPerFrame = 10000;
t = (0:samplesPerFrame-1)/fs;

% Doppler frequency
fd = 2*v*fc/c;

% Pulse parameters
pulsePeriod = 10e-3;             % Pulse repetition interval (10 ms)
pulseWidth = 200e-6;             % Pulse width (200 us)
pulsePeriodSamples = round(pulsePeriod * fs);
pulseSamples = round(pulseWidth * fs);

% Generate pulse envelope
pulseEnvelope = zeros(size(t));
for startIdx = 1:pulsePeriodSamples:length(t)
    endIdx = min(startIdx + pulseSamples - 1, length(t));
    pulseEnvelope(startIdx:endIdx) = 1;
end

% Generate pulsed radar signal
f_pulse = 100e3;   % Baseband pulse tone frequency
pulsedSignal = pulseEnvelope .* exp(1j*2*pi*f_pulse*t);  % Complex pulsed tone
pulsedSignal = pulsedSignal.';  % Column vector

%Transmitter Configuration
tx = sdrtx('Pluto','RadioID', 'usb:0', ...
    'CenterFrequency', fc,'BasebandSampleRate', fs,'Gain', -10);

%Receiver Configuration
rx = sdrrx('Pluto','RadioID', 'usb:0','CenterFrequency', fc, 'BasebandSampleRate', fs, 'SamplesPerFrame', samplesPerFrame,'GainSource', 'Manual', 'Gain', 40);

% Transmit + Receive
transmitRepeat(tx, pulsedSignal);

disp("Running Doppler radar...");

runtime = tic;
i = 0;
while toc(runtime) < 10  % Run for 10 seconds
    i = i + 1;

    % Receive echo
    rxData = rx();

    % Plot time domain
    figure(1);
    plot((t+i)*1e3, real(rxData), 'b'); hold on;
    plot((t+i)*1e3, imag(rxData), 'r'); hold off;
    xlabel('Time (ms)');
    ylabel('Amplitude');
    title('Received Signal - Time Domain');
    legend('Real', 'Imag');
    grid on;
    drawnow;
    pause(0.01);
end

% Cleanup
release(tx);
release(rx);

% Frequency Domain Analysis (Doppler Shift)
N = length(rxData);
f_axis = (-fs/2:fs/N:fs/2 - fs/N)/1e3;  % in kHz
spectrum = fftshift(fft(rxData)/N);
figure(2);
plot(f_axis, abs(spectrum), 'b');
xlabel('Frequency (kHz)');
ylabel('Magnitude');
title('Received Signal - Frequency Domain (Doppler)');
grid on;

% %% MULTIPLE CHIRPS
% %% Doppler / FMCW radar with multiple chirps 
% 
% % -parameters
% fc  = 2.4e9;           % RF centre
% fs  = 1e6;             % BB sample rate
% c   = 3e8;
% M   = 4;               % *** number of chirps per PRI ***
% pulsePeriod = 10e-3;   % 10 ms PRI
% pulseWidth   = 200e-6; % 200 µs per chirp
% f_start      = -200e3; % -200 kHz (relative to BB centre)
% f_end        =  200e3; %  200 kHz
% 
% 
% samplesPerChirp   = round(pulseWidth  * fs);
% samplesPerFrame   = round(pulsePeriod * fs);
% t_frame           = (0:samplesPerFrame-1).'/fs;     % column
% 
% % Build multi‑chirp envelope 
% pulseEnvelope = zeros(samplesPerFrame,1);
% for m = 0:M-1
%     s = m * samplesPerChirp + 1;   % chirp start sample
%     e = min(s + samplesPerChirp - 1, samplesPerFrame);
%     pulseEnvelope(s:e) = 1;
% end
% 
% % Generate one chirp’s complex exponential 
% t_chirp = (0:samplesPerChirp-1).'/fs;
% k       = (f_end - f_start)/pulseWidth;             % sweep rate (Hz/s)
% oneChirp= exp(1j*2*pi*(f_start*t_chirp + 0.5*k*t_chirp.^2));
% 
% %Tile the chirp M times inside the frame 
% pulsedSignal = zeros(samplesPerFrame,1);
% for m = 0:M-1
%     idx = (1:samplesPerChirp) + m*samplesPerChirp;
%     pulsedSignal(idx) = oneChirp;
% end
% pulsedSignal = pulsedSignal .* pulseEnvelope;       % (redundant but explicit)
% 
% % transmit / recieve
% tx = sdrtx('Pluto','CenterFrequency',fc,'BasebandSampleRate',fs,'Gain',-10);
% rx = sdrrx('Pluto','CenterFrequency',fc,'BasebandSampleRate',fs,...
%            'SamplesPerFrame',samplesPerFrame,'GainSource','Manual','Gain',40);
% 
% transmitRepeat(tx,pulsedSignal);
% 
% disp("Running multi‑chirp radar …");
% runtime = tic;
% while toc(runtime) < 10
%     rxData = rx();
% 
%     % Quick range/Doppler visual (slow‑time FFT then fast‑time FFT)
%     data_mat = reshape(rxData(1:M*samplesPerChirp),samplesPerChirp,[]); % [fast × slow]
%     rngFFT   = fft(data_mat,[],1);               % range dimension
%     dopFFT   = fftshift(fft(rngFFT,[],2),2);     % Doppler dimension
% 
%     imagesc(abs(dopFFT)); colormap turbo; colorbar;
%     xlabel('Chirp index (slow‑time)'); ylabel('Range bin (fast‑time)');
%     title('Range‑Doppler map');
%     drawnow;
% end
% release(tx); release(rx);
