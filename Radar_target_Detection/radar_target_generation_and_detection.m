clear all
clc;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%speed of light = 3e8
%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant
 
tr = 110; % target range
tv = -20; % target velocity

radar_max_range = 200;
radar_range_resolution = 1;
radar_max_velocity = 70;
speed_of_light = 3e8;

%% FMCW Waveform Generation

% *%TODO* :
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.


%Operating carrier frequency of Radar 
fc= 77e9;             %carrier frequency given

B = speed_of_light /(2 * radar_range_resolution);                                                   
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
sweep_time = 5.5; %t_sweep
t_chirp = sweep_time * 2 * (radar_max_range/speed_of_light);
slope = B / t_chirp;

Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*t_chirp,Nr*Nd); %total time for samples

%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    
    % *%TODO* :
    %For each time stamp update the Range of the Target for constant velocity. 
    r_t(i) = tr + (tv*t(i));
    td(i) = (2 * r_t(i)) / speed_of_light;
    % *%TODO* :
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) =  cos(2 * pi * (fc * t(i) + 0.5 * slope * t(i)^2));
    Rx (i)  = cos(2 * pi * (fc * (t(i) - td(i)) + 0.5 * slope * (t(i) - td(i))^2));
    
    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i) .* Rx(i);
    
end

%% RANGE MEASUREMENT


 % *%TODO* :
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
Mix = reshape(Mix, [Nr, Nd]);

 % *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
range_fft = fft(Mix, Nr);
 % *%TODO* :
% Take the absolute value of FFT output
range_fft = abs(range_fft);
range_fft = range_fft ./ max(range_fft); % Normalize
 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
range_fft = range_fft(1 : Nr/2-1);

%plotting the range
figure ('Name','Range from First FFT')
subplot(2,1,1)

 % *%TODO* :
 % plot FFT output 
plot(range_fft);
axis ([0 200 0 1]);
title('Range from First FFT');
ylabel('Normalized Amplitude');
xlabel('Range');
 


%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
range_fft2d = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
range_fft2d = range_fft2d(1:Nr/2,1:Nd);
range_fft2d = fftshift (range_fft2d);
RDM = abs(range_fft2d); % radar_doppler_map
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure('NAME','2D-FFT Output Range Doppler Map');
surf(doppler_axis,range_axis,RDM);
colorbar;
title('Output of Range Dopple Map (RDM)');
%% CFAR implementation

%Slide Window through the complete Range Doppler Map

% *%TODO* :
%Select the number of Training Cells in both the dimensions.
tc = 10; %train_cells
tb = 8; %train_band
% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
tg_c = 4; % guard_cells
tg_b = 4; %guard_band
% *%TODO* :
% offset the threshold by SNR value in dB
offset = 1.4;
% *%TODO* :
%Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);


% *%TODO* :
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.

   % Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
   % CFAR

RDM = RDM / max(RDM(:));


for row1 = tc + tg_c + 1 : (Nr/2) - (tc + tg_c)
  for col1 = tb + tg_b + 1 : (Nd) - (tb + tg_b)
    %Create a vector to store noise_level for each iteration on training cells
    noise_level = zeros(1, 1);

    for row2 = row1 - (tc + tg_c) : row1 + (tc + tg_c)
      for col2 = col1 - (tb + tg_b) : col1 + (tb + tg_b)
        if (abs(row1 - row2) > tg_c || abs(col1 - col2) > tg_b)
          noise_level = noise_level + db2pow(RDM(row2, col2));
        end
      end
    end

    % Calculate threshold from noise average then add the offset
    threshold = pow2db(noise_level / (2 * (tb + tg_b + 1) * 2 * (tc + tg_c + 1) - (tg_c * tg_b) - 1));
    threshold = threshold + offset;

    cell_under_test = RDM(row1,col1);

    if (cell_under_test < threshold)
      RDM(row1, col1) = 0;
    else
      RDM(row1, col1) = 1;
    end

  end
end

%RDM(RDM~=0 & RDM~=1) = 0;

% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0.

[lr, lc] = size(RDM);
RDM(union(1:(tc+tg_c), lr-(tc+tg_c-1):lr), :) = 0;  % rows
RDM(:, union(1:(tb+tg_b), lc-(tb+tg_b-1):lc)) = 0;  % columns 


% *%TODO* :
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.

figure('NAME','output of 2D CFAR Process');
surf(doppler_axis, range_axis, RDM);
colorbar;

xlabel('Speed');
ylabel('Range');
zlabel('Normalized Amplitude');
title('Output of 2D CFAR Process');


 
 