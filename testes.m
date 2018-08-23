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

%%%% SIMULANDO INPUTS DA FUNÇÃO
snr = 15
cfgNHT = cfgNHT10
chan = chan10

fs = helperSampleRate(cfgNHT) % Baseband sampling rate

% Parameters used when generating the waveform to be transmitted
numPkts = 1
idleTime = 2.0e-6; 
winTransTime = 1.0e-07

% Indices for accessing each field within the time-domain packet
ind = wlanFieldIndices(cfgNHT);

% Get the number of occupied subcarriers and FFT length
[data,pilots] = helperSubcarrierIndices(cfgNHT, 'Legacy');
Nst = numel(data)+numel(pilots); % Number of occupied subcarriers
% 52 SUBPORTADORAS - BATE COM O PADRÃO
Nfft = helperFFTLength(cfgNHT);     % FFT length

% Create an instance of the AWGN channel per SNR point simulated
AWGN = comm.AWGNChannel;
AWGN.NoiseMethod = 'Signal to noise ratio (SNR)';
AWGN.SignalPower = 1;              % Unit power
AWGN.SNR = snr-10*log10(Nfft/Nst); % Account for energy in nulls

inpPSDU = randi([0 1], cfgNHT.PSDULength*8, 1);

tx = wlanWaveformGenerator(inpPSDU,cfgNHT, 'IdleTime', idleTime,...
        'NumPackets', numPkts, 'WindowTransitionTime', winTransTime);

% COLOCAR OPÇÃO DE CYCLIC PREFIX
% QUANTOS BITS COLOCAR COMO PADDING?
% Add trailing zeros to allow for channel delay
chDelay = 100
padTx = [tx; zeros(chDelay, 1)];

% Pass through HiperLAN/2 fading channel model
rx = step(chan, padTx);
reset(chan);    % Reset channel to create different realizations

% Add noise
rx = step(AWGN, rx);

pktStartIdx = helperPacketDetect(rx, cfgNHT.ChannelBandwidth);
% if isempty(pktStartIdx) % If empty no L-STF detected; packet error
%     numPacketErrors = numPacketErrors+1;
%     numPkt = numPkt+1;
%     continue; % Go to next loop iteration
% end
pktOffset = pktStartIdx-1; % Packet offset from start of waveform

% Extract L-STF and perform coarse frequency offset correction
lstf = rx(pktOffset+(ind.LSTF(1):ind.LSTF(2)),:);