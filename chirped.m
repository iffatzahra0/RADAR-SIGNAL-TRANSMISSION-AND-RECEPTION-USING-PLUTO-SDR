% Radar Parameters
fc = 2.4e9;         % Carrier Frequency (Hz)
B = 10e6;           % Chirp Bandwidth (Hz)
T = 20e-6;          % Chirp Duration (s)
fs = 20e6;          % Sampling Rate (Hz)
c = 3e8;            % Speed of Light (m/s)

% Derived Parameters
N = round(T * fs);                % Samples per chirp
t = (0:N-1)' / fs;                % Time vector
slope = B / T;                    % Chirp slope

% Generate Chirp Signal
chirp = exp(1j * 2 * pi * (0.5 * slope * t.^2));
chirp = chirp / max(abs(chirp)); % Normalize

%% Transmitter Setup (Pluto)
tx = sdrtx('Pluto');
tx.CenterFrequency = fc;
tx.BasebandSampleRate = fs;
tx.Gain = -10;

% Start Transmitting Chirp Repeatedly
transmitRepeat(tx, chirp);

%% Receiver Setup (Pluto)
rx = sdrrx('Pluto');
rx.CenterFrequency = fc;
rx.BasebandSampleRate = fs;
rx.SamplesPerFrame = N;
rx.Gain = 40;
rx.OutputDataType = 'double';
%% plotting 

disp("Running ...");

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
    title('Received chirped Signal - Time Domain');
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
title('Received chirped Signal - Frequency Domain');
grid on;
