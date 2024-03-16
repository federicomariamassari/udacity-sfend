clear, clc;

tic                 % Time the program

%% RADAR SPECIFICATIONS

% Frequency of operation: 77 GHz
% Maximum range: 200 m
% Range resolution: 1 m
% Maximum velocity: 70 m/s

c = 3e8;            % Speed of light, ~300,000 km/s
maxRange = 200;
rangeResolution = 1;
maxVelocity = 70;

%% USER-DEFINED RANGE AND VELOCITY

% Define the target's initial position and (constant) velocity.

R = 110;        % Initial distance of target, [0; 200] m
V = -20;        % Constant speed of target, [-70; +70] m/s

% Assert input validity
assert(R >= 0 & R <= maxRange, 'Initial distance must be in [0; 200] m')
assert(abs(V) <= maxVelocity, 'Speed of target must be in [-70; +70] m/s')

%% FMCW WAVEFORM GENERATION

% Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp), and Slope (slope) of 
% the FMCW chirp using the requirements above.

B = c / 2 * rangeResolution;        % Chirp bandwidth
Tchirp = 5.5 * 2 * maxRange / c;    % Chirp time
slope = B / Tchirp;

fc = 77e9;      % Operating carrier frequency of the radar, 77 GHz

% The number of chirps in one sequence. Its ideal to have 2^(value) for 
% the ease of running the FFT for Doppler Estimation.

Nd = 128;       % No. of Doppler cells or 
                % no. of sent periods / no. of chirps

% The number of samples on each chirp.

Nr = 1024;      % Length of time or no. of range cells

% Timestamp for running the displacement scenario for every sample on 
% each chirp.

t = linspace(0, Nd*Tchirp, Nr*Nd);      % Total time for samples

%% SIGNAL GENERATION AND MOVING TARGET SIMULATION

% Running the radar scenario over the time.

% For each time stamp update the Range of the Target for constant 
% velocity.

r_t = R + V*t;          % Range covered; CVM: d(t) = d(0) + v*t

td = 2*r_t / c;         % Time delay; signal round-trip time (tau)

% For each time sample we need update the transmitted and
% received signal.

Tx = cos(2*pi * (fc*t + (slope*t.^2)/2));   % Transmitted signal

Rx = cos(2*pi * (fc*(t-td) + (slope*(t-td).^2)/2));   % Received signal

% Now by mixing the Transmit and Receive generate the beat signal
% This is done by element wise matrix multiplication of Transmit and
% Receiver Signal.

Mix = Tx.*Rx;           % Beat signal

%% RANGE MEASUREMENT

% Reshape the vector into Nr*Nd array. Nr and Nd here would also define 
% the size of Range and Doppler FFT respectively.

Mix = reshape(Mix, [Nr, Nd]);

% Run the FFT on the beat signal along the range bins dimension (Nr) and
% normalize.
signal_fft = fft(Mix, Nr) / Nr;

% Take the absolute value of FFT output.
signal_fft = abs(signal_fft);

% Output of FFT is double-sided signal, but we are interested in only 
% one side of the spectrum. Hence we throw out half of the samples.
signal_fft = signal_fft(1:Nr/2+1);

figure('Name', 'Range from First FFT')
plot(0:Nr/2, signal_fft);
axis([0 maxRange 0 0.35]);
xlabel('');
ylabel('');

%% RANGE DOPPLER RESPONSE

% The 2D FFT implementation is already provided here. This will run a
% 2D FFT on the mixed signal (beat signal) output and generate a range 
% doppler map. You will implement CFAR on the generated RDM

% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix, Nr, Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2, 1:Nd);
sig_fft2 = fftshift (sig_fft2);

RDM = abs(sig_fft2);
RDM = 10*log10(RDM);

% Use the surf function to plot the output of 2D FFT and to show axis in 
% both dimensions.
doppler_axis = linspace(-maxVelocity, maxVelocity, Nd);
range_axis = linspace(-maxRange, maxRange, Nr/2)*((Nr/2)/(2*maxRange));
figure,surf(doppler_axis, range_axis, RDM);
xlim([-maxVelocity, maxVelocity]);
ylim([-maxRange, maxRange]);

title('Range-Doppler Map', FontSize=14)
xlabel('Doppler', FontSize=12);
ylabel('Range', FontSize=12);
zlabel('Signal', FontSize=12)

%% CA-CFAR IMPLEMENTATION

% Slide Window through the complete Range-Doppler Map.

% Select the number of Training Cells in both the dimensions.

Tr = 10;        % r: Range
Td = 8;         % d: Doppler

%Select the number of Guard Cells in both dimensions around the Cell 
% Under Test (CUT) for accurate estimation.

Gr = 4;
Gd = 4;

% Offset the threshold by Signal-to-Noise Ratio (SNR) value in dB
offset = 6;

% Create a vector to store noise_level for each iteration on training cells

% Design a loop such that it slides the CUT across range doppler map by
% giving margins at the edges for Training and Guard Cells.
% For every iteration sum the signal level within all the training
% cells. To sum convert the value from logarithmic to linear using db2pow
% function. Average the summed values for all of the training
% cells used. After averaging convert it back to logarithimic using pow2db.
% Further add the offset to it to determine the threshold. Next, compare the
% signal under CUT with this threshold. If the CUT level > threshold assign
% it a value of 1, else equate it to 0.

RDM_copy = zeros(size(RDM));
RDM_copy((Tr+Gr+1):end-(Tr+Gr), (Td+Gd+1):end-(Td+Gd)) = ...
    RDM((Tr+Gr+1):end-(Tr+Gr), (Td+Gd+1):end-(Td+Gd));

for i = (Tr+Gr+1):Nr/2-(Gr+Tr)      % As it's calculated on FFT (symmetric spectrum)
    for j = (Td+Gd+1):Nd-(Gd+Td)
        
        noise2 = RDM(i-(Tr+Gr):i+Gr+Tr, j-(Td+Gd):j+Gd+Td);
        noise3 = RDM(i-Gr:i+Gr, j-Gd:j+Gd);
        noise2 = db2pow(noise2);
        noise3 = db2pow(noise3);
        noise_level = sum(noise2(:)) - sum(noise3(:));


        trainingCells = (2*(Tr+Gr)+1)*(2*(Td+Gd)+1) - ((2*Gr+1)*(2*Gd+1));

        threshold = pow2db(noise_level/trainingCells);
        % Add the SNR offset to the threshold (why not multiply?)
        threshold = threshold + offset;
        % Measure the signal in the CUT and compare against
        CUT = RDM(i,j);

        RDM_copy(i,j) = CUT > threshold;
        noise_level = 0;  % Added
    end
end

% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
% than the Range Doppler Map as the CUT cannot be located at the edges of
% matrix. Hence,few cells will not be thresholded. To keep the map size 
% same set those values to 0. 


% *%TODO* :
% Display the CFAR output using the Surf function like we did for Range
% Doppler Response output.
figure, surf(doppler_axis, range_axis, RDM_copy);
xlim([-maxVelocity, maxVelocity]);
ylim([-maxRange, maxRange]);
xlabel('Doppler', FontSize=12);
ylabel('Range', FontSize=12);
colorbar;
 
toc