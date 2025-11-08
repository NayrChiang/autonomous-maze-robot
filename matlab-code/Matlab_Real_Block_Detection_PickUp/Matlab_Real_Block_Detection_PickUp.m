%%
clear
close all
clc

%% Initalization of the world
% dim1 = 32; dim2 = 16; 
% locationindex = reshape(1:dim1*dim2,dim1,dim2)'; 
% n = numel(locationindex);
% rand('twister',5489);
% bw = reshape(randi([0 1],n,1),dim2,dim1); 
% %pattern of the floor

%% Make blocks
% M = zeros(size(bw));
% Blocks = [2, 3; 3, 2; 4, 3; 5, 1; 5, 3; 7, 1; 7, 3; 7, 4;];
% for xx = 1:size(Blocks,1),
% 	x = Blocks(xx,1); y = Blocks(xx,2);
% 	M(1+(y-1)*4:(y-1)*4+4, 1+(x-1)*4:(x-1)*4+4) = 1;
% end

% M = [ones(dim2,1) M ones(dim2,1)];
% M = [ones(1, dim1+2); M; ones(1, dim1+2)];
% %create mask for blocks
% M = abs(M-1);
% M = M(2:end-1, 2:end-1);
% % figure; imagesc((bw+1).*M); colormap(gray);
% %M=mask for obstacle in the maze. 0=obstacle

%% Initialize probability
% p = ones(dim2,dim1)*(1/n); 
% pfig = figure;

%-------------------------------------------------------------------

%% Connect to Bluetooth and process ultrasonic data
serialInfo = serialportlist('available')
s=serialport('COM8',9600);
u = [0,0,0,0,0,0,0,0];
uTemp = [0,0,0,0,0,0,0,0];
fopen(s);
pause(0.5);

%% Robot Sensor Measurements
pos = [0,0,0];  % Position (x,y,rotation)
speed = 0.8 ;
stepCount = 0;
forwardCount = 0;
leftRef = 0;
rightRef = 0;
count = 0;
pickupcount = 0;
blkDetected = 0;
block_num = 17;

%% First Loop
while blkDetected == 0  
    %% Take Measurements
    flush(s);
    sample = fscanf(s);
    split_sample = split(sample);
    strtoint = str2double(split_sample);
    idx = strtoint(~isnan(strtoint));

    if max(size(idx)) == 8
        u = transpose(idx(1:6));
        ir = transpose(idx(7:8));
    end
    distFront = u(1); %IRL #1
    distL45 = u(2);   %IRL #2
    distLeft = u(3);  %IRL #3
    distBack = u(6);  %IRL #6
    distRight = u(4); %IRL #4
    distR45 = u(5);   %IRL #5
    distBottom = ir(2); %IRL IR#2
    disp(u)
    disp(ir)

    %% If the robot enters the loading from Block 17 (Go up from the 4 ft side):
    if count == 0 && block_num == 17
        blkDetected = shiftRight(s, blkDetected);
        if blkDetected == 0
            blkDetected = shiftLeft(s, blkDetected);
            if blkDetected == 0
                fwrite(s,'d')
                pause(0.5);    
                pause(3)
                enterQ4(s);
                pause(2);
                blkDetected = scan_in_Q4(s, blkDetected);  
                if blkDetected == 0
                    blkDetected = scan_in_Q1(s,blkDetected);
                end
            end
        end
        count = 1;
    end

    %% If the robot enters the loading from Block 3 (Go left from the 8 ft side):
    if count == 0 && block_num == 3
        blkDetected = shiftLeft(s, blkDetected);
        if blkDetected == 0
            blkDetected = shiftRight(s, blkDetected);
            if blkDetected == 0
                fwrite(s,'a')
                pause(0.5);
                pause(3);
                enterQ2(s);
                pause(2)
                blkDetected = scan_in_Q1(s, blkDetected);  
                if blkDetected == 0
                    blkDetected = scan_in_Q4(s,blkDetected);
                end
            end
        end
        count = 1;
    end

end

%% Second Loop
%% After robot has detected the load, go forward and pickup the load
while blkDetected == 1 && pickupcount == 0
    %% Get new measurements 
    flush(s);
    sample = fscanf(s);
    split_sample = split(sample);
    strtoint = str2double(split_sample);
    idx = strtoint(~isnan(strtoint));

    if max(size(idx)) == 8
        u = transpose(idx(1:6));
        ir = transpose(idx(7:8));
    end
    
    distFront = u(1); %IRL #1
    distL45 = u(2);   %IRL #2
    distLeft = u(3);  %IRL #3
    distBack = u(6);  %IRL #6
    distRight = u(4); %IRL #4
    distR45 = u(5);   %IRL #5
    distBottom = ir(2); %IRL IR#2    
    
    if distBottom < 16 && pickupcount == 0
        fwrite(s, 'x') % Stop
        pause(2);
        fwrite(s, 'g') % Grab
        pause(4);
        fwrite(s, 'r') % Release

        pickupcount = 1;
    elseif pickupcount == 0
        fwrite(s, 'w') % Go forward
    end
end

fclose;
%% ------------------------------------------------------------------------
%% Supporting Functions

%% Load Detection
function blkDetected = blkScanning(distFront, distBottom)
    if distBottom <= 50 && (abs(distFront-distBottom)) > 10
        blkDetected = 1;
        disp("blk found");
    else
        blkDetected = 0;
    end
end

%% Shift robot to the left until:
    % 1. Too close to the wall. 
    % 2. Detects the load. 
    % The function returns the detection of the load
function blkDetected = shiftLeft(s, blkDetected)
        flush(s);
        sample = fscanf(s);
        split_sample = split(sample);
        strtoint = str2double(split_sample);
        idx = strtoint(~isnan(strtoint));

        if max(size(idx)) == 8
            u = transpose(idx(1:6));
            ir = transpose(idx(7:8));
        end
        distLeft = u(3);
        while(distLeft > 5 && blkDetected == 0)
            flush(s);
            sample = fscanf(s);
            split_sample = split(sample);
            strtoint = str2double(split_sample);
            idx = strtoint(~isnan(strtoint));

            if max(size(idx)) == 8
                u = transpose(idx(1:6));
                ir = transpose(idx(7:8));
            end
            distFront = u(1);
            distLeft = u(3);
            distBack = u(6);
            distBottom = ir(2);
            fwrite(s,'z')
            blkDetected = blkScanning(distFront, distBottom);
        end
    pause(0.5);
end

%% Shift robot to the right
function blkDetected = shiftRight(s, blkDetected)
        flush(s);
        sample = fscanf(s);
        split_sample = split(sample);
        strtoint = str2double(split_sample);
        idx = strtoint(~isnan(strtoint));

        if max(size(idx)) == 8
            u = transpose(idx(1:6));
            ir = transpose(idx(7:8));
        end
        distRight = u(4);
        while(distRight > 5 && blkDetected == 0)
            flush(s);
            sample = fscanf(s);
            split_sample = split(sample);
            strtoint = str2double(split_sample);
            idx = strtoint(~isnan(strtoint));

            if max(size(idx)) == 8
                u = transpose(idx(1:6));
                ir = transpose(idx(7:8));
            end

            distFront = u(1);
            distRight = u(4);
            distBack = u(6);
            distBottom = ir(2);
            fwrite(s, 'c')
            blkDetected = blkScanning(distFront, distBottom);
        end
    pause(0.5);
end

%% Shift left until front distance > threshold
function enterQ4(s)
        flush(s);
        sample = fscanf(s);
        split_sample = split(sample);
        strtoint = str2double(split_sample);
        idx = strtoint(~isnan(strtoint));

        if max(size(idx)) == 8
            u = transpose(idx(1:6));
            ir = transpose(idx(7:8));
        end
        distFront = u(1);
        while(distFront < 20)
            flush(s);
            sample = fscanf(s);
            split_sample = split(sample);
            strtoint = str2double(split_sample);
            idx = strtoint(~isnan(strtoint));

            if max(size(idx)) == 8
                u = transpose(idx(1:6));
                ir = transpose(idx(7:8));
            end

            distFront = u(1);
            distBack = u(6);
            fwrite(s, 'z')
        end
    fwrite(s, 'z')
    pause(0.5);
end

%% Shift left until front distance > threshold
function enterQ2(s)
         flush(s);
         sample = fscanf(s);
         split_sample = split(sample);
         strtoint = str2double(split_sample);
         idx = strtoint(~isnan(strtoint));

            if max(size(idx)) == 8
                u = transpose(idx(1:6));
                ir = transpose(idx(7:8));
            end
         distFront = u(1);
        while(distFront < 20)
            flush(s);
            sample = fscanf(s);
            split_sample = split(sample);
            strtoint = str2double(split_sample);
            idx = strtoint(~isnan(strtoint));

            if max(size(idx)) == 8
                u = transpose(idx(1:6));
                ir = transpose(idx(7:8));
            end

            distFront = u(1);
            distBack = u(6);
            fwrite(s, 'c')
        end
    fwrite(s, 'c')
    pause(0.5);
end

%% Entering Q3/Q4
    % Rotate 90 degrees while checking if the load can be detected
function blkDetected = scan_in_Q4(s, blkDetected)
    angle = 0; %set initial angle;
    while (blkDetected == 0 && angle < 105)
        flush(s);
        sample = fscanf(s);
        split_sample = split(sample);
        strtoint = str2double(split_sample);
        idx = strtoint(~isnan(strtoint));

        if max(size(idx)) == 8
            u = transpose(idx(1:6));
            ir = transpose(idx(7:8));
        end

        fwrite(s, 'q') % rotate by 5deg clockwise
        angle = angle + 5;
        distBottom = ir(2);
        distFront = u(1);
        blkDetected = blkScanning(distFront,distBottom); %check for the block
    end
end

%% Entering Q1
function blkDetected = scan_in_Q1(s, blkDetected)
    angle = 0; %set initial angle;
        while (blkDetected == 0 && angle < 105)
            flush(s);
            sample = fscanf(s);
            split_sample = split(sample);
            strtoint = str2double(split_sample);
            idx = strtoint(~isnan(strtoint));

            if max(size(idx)) == 8
                u = transpose(idx(1:6));
                ir = transpose(idx(7:8));
            end

            fwrite(s, 'e') % rotate by 5deg clockwise
            angle = angle + 5;
            distBottom = ir(2);
            distFront = u(1);
            blkDetected = blkScanning(distFront,distBottom); %check for the block
        end
end











