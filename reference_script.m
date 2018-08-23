% Link parameters
mcs = 4;                % QPSK rate 1/2
psduLen = 1000;         % PSDU length in bytes

% Create a format configuration object for a 802.11p transmission
cfgNHT10 = wlanNonHTConfig;
cfgNHT10.ChannelBandwidth = 'CBW10';    % 10 MHz channel bandwidth
cfgNHT10.PSDULength = psduLen;
cfgNHT10.MCS = mcs;

% Create a format configuration object for a 802.11a transmission
cfgNHT20 = wlanNonHTConfig;
cfgNHT20.ChannelBandwidth = 'CBW20';    % 20 MHz channel bandwidth
cfgNHT20.PSDULength = psduLen;
cfgNHT20.MCS = mcs;

% Create and configure the channel
fd = 50;                            % Maximum Doppler shift, Hz
c = 3e8*3.6;                        % Speed of light, Km/hr
fc = 5.9e9;                         % Carrier frequency, Hz
disp(['Speed of unit = ' num2str(c*fd/5.9e9) ' Km/hr at ' num2str(fc/1e9) ' GHz']);

fs20 = helperSampleRate(cfgNHT20);    % Baseband sampling rate for 20 MHz

% Path delays (s) and average path gains (dB) for HiperLAN/2 Model-E
PathDelays = [0 10 20 40 70 100 140 190 240 320 430 560 710 880 1070 ...
    1280 1510 1760] * 1e-9;
AveragePathGains = [-4.9 -5.1 -5.2 -0.8 -1.3 -1.9 -0.3 -1.2 -2.1 0.0 ...
    -1.9 -2.8 -5.4 -7.3 -10.6 -13.4 -17.4 -20.9];

chan20 = comm.RayleighChannel(...
            'PathDelays',           PathDelays, ...
            'AveragePathGains',     AveragePathGains, ...
            'SampleRate',           fs20,...
            'MaximumDopplerShift',  fd);

fs10 = helperSampleRate(cfgNHT10);    % Baseband sampling rate for 10 MHz
chan10 = comm.RayleighChannel(...
            'PathDelays',           PathDelays, ...
            'AveragePathGains',     AveragePathGains, ...
            'SampleRate',           fs10, ...
            'MaximumDopplerShift',  fd);
        
fadingMargin = 12; % dB
snrOperatingVec = [-1 1.75 2 4.75 7.5 10.75 15 16.5] + fadingMargin;
simRange = -2:1:1;
snr = snrOperatingVec(mcs+1) + simRange;

enableFE = false;    % Disable front-end receiver components

maxNumErrors = 20;   % The maximum number of packet errors at an SNR point
maxNumPackets = 200; % Maximum number of packets at an SNR point

% Set random stream for repeatability of results
s = rng(98765);

% Set up a figure for visualizing PER results
h = figure;
grid on;
hold on;
ax = gca;
ax.YScale = 'log';
xlim([snr(1), snr(end)]);
ylim([1e-3 1]);
xlabel('SNR (dB)');
ylabel('PER');
h.NumberTitle = 'off';
h.Name = '802.11p vs. 802.11a PER';
title(['MCS ' num2str(mcs) ', HIPERLAN/2 Model E' ...
       ', Doppler ' num2str(fd) ' Hz']);

% Simulation loop for both links
S = numel(snr);
per20 = zeros(S,1);
per10 = per20;
for i = 1:S
    % 802.11p link
    per10(i) = nonHTPERSimulator(cfgNHT10, chan10, snr(i), ...
        maxNumErrors, maxNumPackets, enableFE);

    % 802.11a link
    per20(i) = nonHTPERSimulator(cfgNHT20, chan20, snr(i), ...
        maxNumErrors, maxNumPackets, enableFE);

    % Compare
    semilogy(snr, per10, 'bx-');
    semilogy(snr, per20, 'ro-');
    legend('802.11p, 10 MHz', '802.11a, 20 MHz');
    drawnow;
end
axis([5 35 5e-3 1])
hold off;

% Restore default stream
rng(s);

