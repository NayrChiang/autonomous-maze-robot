clear
close all
clc

%% Initalization of the world
dim1 = 32; dim2 = 16; 
locationindex = reshape(1:dim1*dim2,dim1,dim2)'; 
n = numel(locationindex);
rand('twister',5489);
bw = reshape(randi([0 1],n,1),dim2,dim1); 
%pattern of the floor

%Make blocks
M = zeros(size(bw));
Blocks = [2, 3; 3, 2; 4, 3; 5, 1; 5, 3; 7, 1; 7, 3; 7, 4;];
for xx = 1:size(Blocks,1),
	x = Blocks(xx,1); y = Blocks(xx,2);
	M(1+(y-1)*4:(y-1)*4+4, 1+(x-1)*4:(x-1)*4+4) = 1;
end
M = [ones(dim2,1) M ones(dim2,1)];
M = [ones(1, dim1+2); M; ones(1, dim1+2)];
%create mask for blocks
M = abs(M-1);
M = M(2:end-1, 2:end-1);
% figure; imagesc((bw+1).*M); colormap(gray);
%M=mask for obstacle in the maze. 0=obstacle

%initialize probability
p = ones(dim2,dim1)*(1/n); 
pfig = figure;

%% Connect to Bluetooth and process ultrasonic data
serialInfo = serialportlist('available')
s = serialport('COM8',9600);
fopen(s); % BT Initialization
pause(0.5);

%% Robot Parameters
u = [0,0,0,0,0,0];  % Ultrasonic measurements 
uTemp = [0,0,0,0,0,0]; 
pos = [0,0,0];  % Position (x,y,rotation)
stepCount = 0;
forwardCount = 0;
leftRef = 0;
rightRef = 0;
count = 0;
stepbackcount = 0;
i_c = 0;
i_done = 0;

% Probability Maxtrix's Size
row_n = size(p, 1); 
col_n = size(p, 2);

% Fixed path info, Each element consist of:
% Block Number / Heading (loading) / Destination % (loading, just for reference) / Heading (unloading) / Destination (unloading)
% When used in the function, use variable(index).split() and get an array of strings
all_path = ["1-0-Self-270-25"; "2-180-1-180-1"; "3-180-2-90-4"; "4-180-2-270-12"; "5----"; "6-270-14-90-Self"; "7----"; "8-270-16-90-Self"; "9-90-1-270-25"; "10-90-2-180-9"; "11----"; "12-90-4-0-14"; "13-180-12-0-14"; "14-180-12-90-6"; "15-180-12-0-16"; "16-180-12-90-8"; "17-90-9-270-25"; "18----"; "19-270-27-90-Self"; "20----"; "21----"; "22-270-30-90-14"; "23----"; "24-90-16-270-32"; "25-90-9-0-27"; "26-180-25-0-27"; "27-180-25-90-19"; "28-180-25-180-27"; "29-180-25-180-27"; "30-180-25-90-14"; "31----"; "32-90-16-270-Self"];

% Adjustable Parameters
heading = 0;
load_unload = 0; % Loading = 0, Unloading = 1, Pick Up Block = 2
unloading = 6; % Specify unloading zone
threshold = 0.1;

% Wall Distance for obstacle avoidance
wallFront = 8;
wallLeft = 4;
wallRight = 4;
wallL45 = 5;
wallR45 = 5;
wallBack = 5;

%% Adjust all_path for unloading zone
all_path = path_adjust(all_path, unloading); 

%% Main Loop
while 1
	%% Take Measurements
    % flush(s);
    sample = fscanf(s);
    split_sample = split(sample);
    strtoint = str2double(split_sample);
    idx = strtoint(~isnan(strtoint));
    
    if max(size(idx)) == 8
        u = transpose(idx(1:7));
        ir = transpose(idx(8));
    end
    
    %% Display Values
    % fprintf('IR Sensor: %dblack\n', ir);
    % fprintf('IR Sensor: %dwhite\n', ir);   
    % fprintf('Compass reading: %d\n', u);
    disp('Heading: ' + string(heading))
    disp(u)

    %% Sensor values 
    distFront = u(1); 
    distL45 = u(2); 
    distLeft = u(3); 
    distRight = u(4);     
    distR45 = u(5); 
    distBack = u(6); 
    

    %% Update Probability
    p = sense_r(bw, M, p, ir);
    figure(pfig)
    imagesc(p);
    title(['step: ' num2str(stepCount)]);
    
    %% Threshold Probability value to start using fixed path
    % Using max value for now
    [p_max,I] = max(p(:));
    [p_row, p_col] = ind2sub(size(p),I);
    disp_pmax = ['Max Probability: ' + string(p_max)];
    disp(disp_pmax)
    
    %% Check if the robot is at 4 way
    % 0 and 180 degree, front and rear has the same range of values    
    if ( ( 60 <= distFront && distFront <= 110) && (  60 <= distBack && distBack <= 110) ) && i_done == 0
        if ( ( 30 <= distLeft && distLeft <= 60) && ( 70 <= distRight && distRight <= 95) )
            heading = 0;
            i_c = 1;
            if i_done == 0
                p = ones(dim2,dim1)*(1/n);
                i_done = 1;
                %% Turn to 180 direction
                heading = turn_2_next_block(s, heading, 180);               
            end            
            disp('4 Way Heading Update to ' + string(heading))
        elseif ( ( 30 <= distRight && distRight <= 60) && ( 70 <= distLeft && distLeft <= 95) ) 
            heading = 180;
            i_c = 1;
            if i_done == 0
                p = ones(dim2,dim1)*(1/n);
                i_done = 1;
                %% Already in 180 direction 
            end
            disp('4 Way Heading Update to ' + string(heading))
        end
    % 90 and 270 degree, sides has the same range of values
    elseif ( ( 70 <= distLeft && distLeft <= 95) && ( 70 <= distRight && distRight <= 95) ) && i_done == 0
        if ( ( 25 <= distFront && distFront <= 70) && ( 60 <= distBack && distBack <= 100) )
            heading = 90;
            i_c = 1;
            if i_done == 0
                p = ones(dim2,dim1)*(1/n);
                i_done = 1;
                %% Turn to 270 direction
                heading = turn_2_next_block(s, heading, 180); 
            end
            disp('4 Way Heading Update to ' + string(heading))
        elseif ( ( 25 <= distBack && distBack <= 70) && ( 60 <= distFront && distFront <= 100) )
            heading = 270; 
            i_c = 1;
            if i_done == 0
                p = ones(dim2,dim1)*(1/n);
                i_done = 1;
                %% Already in 270 direction
            end
            disp('4 Way Heading Update to ' + string(heading))
        end
    end    
    
    %% Obstacle Avoidance with Fixed Path:
    if  p_max*i_c > threshold % high_p_max > th_sum 
        disp('Begin Following Fixed Path')        
        % Determine Path
        cur_block = block_finder(p_row, p_col); % Current block in string
        cur_block_n = str2num(cur_block); % Current block in int
        disp_cur_b = ["Current block: " + cur_block]; 
        disp(disp_cur_b) % print current block info
        path_info = all_path(cur_block_n); % get path info
        % 1 x 5 array
        path_info_a = path_info.split('-'); % get path info as an array
        
        %% Going to Loading Zone
        if load_unload == 0
        %% If in the loading zone already
            if sum(find( cur_block == ["1", "2", "9", "10"] > 0)) > 0
                if distFront >= wallFront
                    [p, heading] = forward_load_unload(s, distLeft, distRight, distL45, distR45, wallLeft, wallRight, wallL45, wallR45, heading, p, M);
                    %% Angle correction logic
                    angle_correction(s, distLeft, distRight, uTemp);
                else
                    disp("-------------------------")
                    disp("I am at the loading zone!")
                    disp("-------------------------")
                    % Change to following unloading path and adjust heading
                    % based on unloading zone
                    dest_heading = str2num(path_info_a(4));
                    heading_change = (dest_heading - heading); % only possible choice -3, -2, -1, 0, 1, 2, 3
                    load_unload = 1;
                    %% Heading to unloading zone, turning to the exit (Turn to the next block function)
                    heading = turn_2_next_block(s, heading, heading_change);
                end

            %% If in the right vertical corridor, exit the corridor when the robot is in block 16
            elseif cur_block == "16"
                % CHANGE DISTANCE VALUE!
                if (heading == 90 && distFront <= 17) || (heading == 270 && distFront <= 29)  
                    dest_heading = 180;
                    heading_change = (dest_heading - heading);
                    %% Turning to the next block
                    heading = turn_2_next_block(s, heading, heading_change);
                else
                    [p, heading] = obstacle_avoidance_fp(s, distLeft, distRight, distFront, distL45, distR45, wallFront, wallLeft, wallRight, wallL45, wallR45, heading, uTemp, p, M, path_info_a, load_unload);
                end                    
                
            %% In other blocks
            else 
                %% Move robot:
                % Updates the heading only when the robot is close to the wall
                [p, heading] = obstacle_avoidance_fp(s, distLeft, distRight, distFront, distL45, distR45, wallFront, wallLeft, wallRight, wallL45, wallR45, heading, uTemp, p, M, path_info_a, load_unload);
            end
        
        %% Block Picking Movement/code
                
        %% Going to Unloading Zone
        else
            %% For Block 27
            % Block 19 is a special unloadig zone since the robot needs to turn
            % up with the front distance ~96 cm at block 27
            dest_heading = str2num(path_info_a(4));
            if cur_block_n == 27 && unloading == 19 && heading ~= 90
                % CHANGE DISTANCE VALUE!
                if distFront <= 29 || distFront <= 41
                    heading_change = (dest_heading - heading);
                    %% Turning to the next block
                    heading = turn_2_next_block(s, heading, heading_change);
                else
                    [p, heading] = forward_load_unload(s, distLeft, distRight, distL45, distR45, wallLeft, wallRight, wallL45, wallR45, heading, p, M);
                    %% Angle correction logic 
                    angle_correction(s, distLeft, distRight, uTemp);
                end
                
            %% Adjust the heading when the robot is in the bottom corridor & missed block 19
            elseif sum(find( cur_block == ["26", "28", "29"] > 0)) > 0 && unloading == 19 && heading ~= dest_heading
                heading_change = (dest_heading - heading);
                %% Turning to the next block
                heading = turn_2_next_block(s, heading, heading_change);                
            
            %% In unloading zone, drive to the middle of the block 
            elseif cur_block_n == unloading
                if distFront >= wallFront
                    [p, heading] = forward_load_unload(s, distLeft, distRight, distL45, distR45, wallLeft, wallRight, wallL45, wallR45, heading, p, M);
                    %% Angle correction logic
                    angle_correction(s, distLeft, distRight, uTemp);
                else
                    disp("-------------------------")
                    disp("I am at the unloading zone!")
                    disp("-------------------------")
                end
                
            %% In other blocks
            else
                %% Motor control when following Fixed Path & Not in Loading/Unloading:
                [p, heading] = obstacle_avoidance_fp(s, distLeft, distRight, distFront, distL45, distR45, wallFront, wallLeft, wallRight, wallL45, wallR45, heading, uTemp, p, M, path_info_a, load_unload);
            end
        end
                
    %% If the probability is not high enough, just do obstacle avoidance    
    else                
        %% Motor control original:
        % Obstacle Avoidance including Angle Correction
        [p, heading] = obstacle_avoidance(s, distLeft, distRight, distFront, distL45, distR45, wallFront, wallLeft, wallRight, wallL45, wallR45, heading, uTemp, p, M);       
    end
disp('-------------')
% pause;

end

%% ------------------------------------------------------------------------
%% Supporting Functions

%% Path Adjustment
function p_adj = path_adjust(all_path, unloading)
% Input = row and column number
% Output = Block number in string
% Example
% all_path = path_adjust(all_path, unloading); 

% Use lower path
if unloading == 6
    all_path(1) = "1-180-Self-270-25"; % 1 go down to 25
    all_path(2) = "2-180-1-180-1"; % 2 go left to 1
    all_path(9) = "9-90-1-270-25"; % 9 go down to 25
    all_path(10) = "10-90-2-180-9"; % 10 go left to 9
    all_path(14) = "14-180-12-90-6"; % 14 go up to 6
    all_path(19) = "19-270-27-270-27"; % 19 go down to 27
    all_path(30) = "30-180-25-90-6"; % 30 go up to 6
    % loop around
    all_path(4) = "4-180-2-180-1"; % 4 go left to 1
    all_path(12) = "12-90-4-90-4"; % 12 go up to 4
    all_path(16) = "16-180-12-180-12"; % 16 go left 12
elseif unloading == 19
    all_path(1) = "1-180-Self-270-25"; % 1 go down to 25
    all_path(2) = "2-180-1-180-1"; % 2 go left to 1
    all_path(9) = "9-90-1-270-25"; % 9 go down to 25
    all_path(10) = "10-90-2-180-9"; % 10 go left to 9
    all_path(6) = "6-270-14-270-30"; % 6 go down to 30
    all_path(19) = "19-270-27-90-Self"; % 19 go up to wall
    all_path(30) = "30-180-25-180-25"; % 30 go left to 25
    % loop around
    all_path(4) = "4-180-2-180-1"; % 4 go left to 1
    all_path(12) = "12-90-4-90-4"; % 12 go up to 4
    all_path(16) = "16-180-12-180-12"; % 16 go left 12
% Use upper path
elseif unloading == 8
    all_path(1) = "1-180-Self-0-4"; % 1 go right to 4
    all_path(2) = "2-180-1-0-4"; % 2 go left to 4
    all_path(9) = "9-90-1-90-1"; % 9 go up to 1
    all_path(10) = "10-90-2-90-2"; % 10 go up to 2
    all_path(16) = "16-180-12-90-8"; % 16 go up to 8
    % loop around
    all_path(6) = "6-270-14-270-30"; % 6 go down to 30 
    all_path(25) = "25-90-9-0-27";  % 25 go up to 1
    all_path(30) = "30-180-25-90-14"; % 30 go left to 25
elseif unloading == 32
    all_path(1) = "1-180-Self-0-4"; % 1 go right to 4
    all_path(2) = "2-180-1-0-4"; % 2 go left to 4
    all_path(9) = "9-90-1-90-1"; % 9 go up to 1
    all_path(10) = "10-90-2-90-2"; % 10 go up to 2
    all_path(16) = "16-180-12-270-32"; % 16 go down to 32
    % loop around
    all_path(6) = "6-270-14-270-30"; % 6 go down to 30 
    all_path(25) = "25-90-9-0-27";  % 25 go up to 1
    all_path(30) = "30-180-25-90-14"; % 30 go left to 25
end
% Return the all_path data
p_adj = all_path;
end

%% Finding the block number
function b_f = block_finder(row_n, col_n)
% Input = row and column number
% Output = Block number in string

cur_block = '';

% Row 1
if 1 <= row_n && row_n <= 4
    if 1 <= col_n && col_n <= 4
        cur_block = "1";
    elseif 5 <= col_n && col_n <= 8
        cur_block = "2";
    elseif 9 <= col_n && col_n <= 12
        cur_block = "3";
    elseif 13 <= col_n && col_n <= 16
        cur_block = "4";
    elseif 17 <= col_n && col_n <= 20
        cur_block = "5";
    elseif 21 <= col_n && col_n <= 24
        cur_block = "6";
    elseif 25 <= col_n && col_n <= 28
        cur_block = "7";
    elseif 29 <= col_n && col_n <= 32
        cur_block = "8";
    end
% Row 2
elseif 5 <= row_n && row_n <= 8
    if 1 <= col_n && col_n <= 4
        cur_block = "9";
    elseif 5 <= col_n && col_n <= 8
        cur_block = "10";
    elseif 9 <= col_n && col_n <= 12
        cur_block = "11";
    elseif 13 <= col_n && col_n <= 16
        cur_block = "12";
    elseif 17 <= col_n && col_n <= 20
        cur_block = "13";
    elseif 21 <= col_n && col_n <= 24
        cur_block = "14";
    elseif 25 <= col_n && col_n <= 28
        cur_block = "15";
    elseif 29 <= col_n && col_n <= 32
        cur_block = "16";
    end  
% Row 3
elseif 9 <= row_n && row_n <= 12
    if 1 <= col_n && col_n <= 4
        cur_block = "17";
    elseif 5 <= col_n && col_n <= 8
        cur_block = "18";
    elseif 9 <= col_n && col_n <= 12
        cur_block = "19";
    elseif 13 <= col_n && col_n <= 16
        cur_block = "20";
    elseif 17 <= col_n && col_n <= 20
        cur_block = "21";
    elseif 21 <= col_n && col_n <= 24
        cur_block = "22";
    elseif 25 <= col_n && col_n <= 28
        cur_block = "23";
    elseif 29 <= col_n && col_n <= 32
        cur_block = "24";
    end     
% Row 4
elseif 13 <= row_n && row_n <= 16
    if 1 <= col_n && col_n <= 4
        cur_block = "25";
    elseif 5 <= col_n && col_n <= 8
        cur_block = "26";
    elseif 9 <= col_n && col_n <= 12
        cur_block = "27";
    elseif 13 <= col_n && col_n <= 16
        cur_block = "28";
    elseif 17 <= col_n && col_n <= 20
        cur_block = "29";
    elseif 21 <= col_n && col_n <= 24
        cur_block = "30";
    elseif 25 <= col_n && col_n <= 28
        cur_block = "31";
    elseif 29 <= col_n && col_n <= 32
        cur_block = "32";
    end      
end        
b_f = cur_block;
end

%% Obstacle Avoidance including Angle Correction (Original Obs Avoid)
function [p, heading] = obstacle_avoidance(s, distLeft, distRight, distFront, distL45, distR45, wallFront, wallLeft, wallRight, wallL45, wallR45, heading, uTemp, p, M)
% 
%% Motor control original:
if (distFront >= wallFront)
    if ( distLeft > wallLeft && distRight > wallRight && distL45 > wallL45 && distR45 > wallR45 )
        %% all clear, move forward. Every three steps count as one
        fwrite(s,'w')
        disp("1 foward, w")
        %call move function to update probability
        p = move(p, M, heading);
        
    %% Adjustment
    else 
        %% Small Adjusment
        if ( distLeft > distRight )
            fwrite(s, 'q')
            %fwrite(s, 'f')
            disp('1 towards left, q')
        else 
            fwrite(s,'e')
            %fwrite(s, 'f')
            disp('1 towards right, e')
        end
        disp("Stepback")
    end
        
    %% Angle correction logic
    rightRef = distRight; 
    leftRef = distLeft; 
    % Take side measurements for correction
    for ct = 3:4 
        flush(s);
        sample = fscanf(s);
        split_sample = split(sample);
        strtoint = str2double(split_sample);
        idx = strtoint(~isnan(strtoint));

        if max(size(idx)) == 8
            u = transpose(idx(3:4));
            uTemp(ct) = u(ct-2);
        end
    end
    leftTemp = uTemp(3); 
    rightTemp = uTemp(4); 
    
    %% Use the side that has smaller distance to compare
    if ( rightTemp <= leftTemp )
        if ( rightTemp > rightRef )
            %robot drifting towards left, correct towards
            %right by 5 degrees
            fwrite(s,'e')
            disp("Correction towards right, e")
        elseif ( rightTemp < rightRef )
            %robot drifting towards right, correct towards
            %left by 5 degrees
            fwrite(s,'q')
            disp("Correction towards left, q")
        else
            disp("No correction")
        end
    else % Left < right
        if ( leftTemp > leftRef )
            fwrite(s,'q')
            disp("Correction towards left, q")
        elseif (leftTemp < leftRef)
            fwrite(s,'e')
            disp("Correction towards right, e") 
        else 
            disp("No correction")
        end
    end
    %end of angle correction logic
    
%% Front is blocked, turn to the side with larger room
else % distFront < 4. The front is blocked.
    if ( distLeft > distRight )
        %turn to the side with larger clearance
        fwrite(s,'a')
        disp("4 left, a")
        heading = mod((heading + 90),360); %left turn
    else % distRight > distLeft
        fwrite(s,'d')
        disp("5 right, d")
        heading = mod((heading - 90),360); %right turn
    end
end
% pause;
end

%% Angle correction logic
function angle_correction(s, distLeft, distRight, uTemp)
rightRef = distRight; 
leftRef = distLeft; 
% Take side measurements for correction
for ct = 3:4 
    flush(s);
    sample = fscanf(s);
    split_sample = split(sample);
    strtoint = str2double(split_sample);
    idx = strtoint(~isnan(strtoint));

    if max(size(idx)) == 8
        u = transpose(idx(3:4));
        uTemp(ct) = u(ct-2);
    end
end
leftTemp = uTemp(3); 
rightTemp = uTemp(4); 

%% Use the side that has smaller distance to compare
if ( rightTemp <= leftTemp )
    if ( rightTemp > rightRef )
        %robot drifting towards left, correct towards
        %right by 5 degrees
        fwrite(s,'e')
        disp("Correction towards right, e")
    elseif ( rightTemp < rightRef )
        %robot drifting towards right, correct towards
        %left by 5 degrees
        fwrite(s,'q')
        disp("Correction towards left, q")
    else
        disp("No correction")
    end
else % Left < right
    if ( leftTemp > leftRef )
        fwrite(s,'q')
        disp("Correction towards left, q")
    elseif (leftTemp < leftRef)
        fwrite(s,'e')
        disp("Correction towards right, e") 
    else 
        disp("No correction")
    end
end
%end of angle correction logic
end

%% Obstacle Avoidance when following Fixed Path & Not in Loading/Unloading:
function [p, heading] = obstacle_avoidance_fp(s, distLeft, distRight, distFront, distL45, distR45, wallFront, wallLeft, wallRight, wallL45, wallR45, heading, uTemp, p, M, path_info_a, load_unload)
%  
if (distFront >= wallFront)
    if ( distLeft > wallLeft && distRight > wallRight && distL45 > wallL45 && distR45 > wallR45 )
        %% all clear, move forward. Every three steps count as one
        fwrite(s,'w')
        disp("1 foward, w")
        %call move function to update probability
        p = move(p, M, heading);
        
    %% Adjustment
    else 
        %% Small Adjusment
        if ( distLeft > distRight )
            fwrite(s, 'q')
            %fwrite(s, 'f')
            disp('1 towards left, q')
        else 
            fwrite(s,'e')
            %fwrite(s, 'f')
            disp('1 towards right, e')
        end
        disp("Stepback")
    end
        
    %% Angle correction logic
    rightRef = distRight; 
    leftRef = distLeft; 
    % Take side measurements for correction
    for ct = 3:4 
        flush(s);
        sample = fscanf(s);
        split_sample = split(sample);
        strtoint = str2double(split_sample);
        idx = strtoint(~isnan(strtoint));

        if max(size(idx)) == 8
            u = transpose(idx(3:4));
            uTemp(ct) = u(ct-2);
        end
    end
    leftTemp = uTemp(3); 
    rightTemp = uTemp(4); 
    
    %% Use the side that has smaller distance to compare
    if ( rightTemp <= leftTemp )
        if ( rightTemp > rightRef )
            %robot drifting towards left, correct towards
            %right by 5 degrees
            fwrite(s,'e')
            disp("Correction towards right, e")
        elseif ( rightTemp < rightRef )
            %robot drifting towards right, correct towards
            %left by 5 degrees
            fwrite(s,'q')
            disp("Correction towards left, q")
        else
            disp("No correction")
        end
    else % Left < right
        if ( leftTemp > leftRef )
            fwrite(s,'q')
            disp("Correction towards left, q")
        elseif (leftTemp < leftRef)
            fwrite(s,'e')
            disp("Correction towards right, e") 
        else 
            disp("No correction")
        end
    end
    %end of angle correction logic
   
%% Front is blocked, Adjust Robot Heading to dest_heading
else % distFront < 4. The front is blocked.
    % Change the parameter for getting the dest_heading in path_info_a
    if load_unload == 0
        path_info_a_n = 2;
    else
        path_info_a_n = 4;
    end
    dest_heading = str2num(path_info_a(path_info_a_n));
    heading_change = (dest_heading - heading); % only possible choice -3, -2, -1, 0, 1, 2, 3
    %% Turning to the next block
    heading = turn_2_next_block(s, heading, heading_change);
end
% pause;
end

%% Turn to the next block
function heading = turn_2_next_block(s, heading_0, heading_change)
%
heading = heading_0;
if heading_change == 270 || heading_change == -90
    % Turn right x 1
    fwrite(s,'d')    
    heading = mod((heading - 90),360); %right turn;
    disp("T2B right, d")                       
elseif heading_change == -180 || heading_change == 180
    % Turn right x 2
    fwrite(s,'dd') 
    heading = mod((heading - 180),360); %Turn Around;
    disp("T2B Turn 180")
elseif heading_change == 90 || heading_change == -270
    % Turn left x 1
    fwrite(s,'a') 
    heading = mod((heading + 90),360); %left turn;
    disp("T2B Turn left x1")                    
else % Heading same and destination heading, no change
end
end

%% Move forward in loading/unloading
function [p, heading] = forward_load_unload(s, distLeft, distRight, distL45, distR45, wallLeft, wallRight, wallL45, wallR45, heading, p, M)
%  
%% All clear, move forward
if ( distLeft > wallLeft && distRight > wallRight && distL45 > wallL45 && distR45 > wallR45 )
    %all clear, move forward. Every  three steps count as one
    %movement
    fwrite(s,'w')
    disp("1 foward, w")
    %call move function to update probability
    p = move(p, M, heading);

%% Adjustment
else
    %% Small Adjusment
    if ( distL45 > distR45 )
        fwrite(s, 'q')
        %fwrite(s, 'f')
        disp('1 towards left, q')
    else 
        fwrite(s,'e')
        %fwrite(s, 'f')
        disp('1 towards right, e')
    end
    disp("Stepback")
end
end
%% Check if the load is straight ahead
function blkDetected = blkScanning(distFront, distBottom)
    if distBottom <= 48 && (abs(distFront-distBottom)) > 1
        blkDetected = 1;
        disp(distBottom);
    else
        blkDetected = 0;
    end
end

%% Shift the robot to the left until reaching the wall OR detecting a block
%returns blkDetected = 1 if detecting the load
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
        fwrite(s,'z') %shift to the left
        blkDetected = blkScanning(distFront, distBottom);
%         correctShiftAngle(s, distBack);
    end
    pause(0.5);
end

%% Shift the robot to the right unitl reaching the wall OR detecting a block
%returns blkDetected = 1 if detecting the load
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
        fwrite(s, 'c') %shift to the right
        blkDetected = blkScanning(distFront, distBottom);
%         correctShiftAngle(s, distBack);
    end
    pause(0.5);
end
%% Shift into quadrant 4 of the loading zone for scanning the load
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
    while(distFront < 16) %shift to the left while the front sensor is blocked by the wall
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
%         correctShiftAngle(s, distBack);
    end
    fwrite(s, 'zz') %shift two more steps after the front is clear
    pause(0.5);
end
%% Shift into quadrant 1 of the loadinf zone for scanning the load
function enterQ1(s)
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
    while(distFront < 16) %shift to the right while front is blocked by the wall
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

%         correctShiftAngle(s, distBack);
    end
    fwrite(s, 'cc')
    pause(0.5);
end
%% Scan for the load when the robot is in quadrant 4
%returns blkDetected = 1 and terminates if finding the block
function blkDetected = scan_in_Q4(s, blkDetected)
    angle = 0; %set initial angle;
        while (blkDetected == 0 && angle < 105) %rotate 105deg in total (account for under/over turning)
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
            blkDetected = blkScanning(distBottom,distFront); %scan for the block
        end

end
%% Scan for the load when the robot is in quadrant 2
%returns blkDetected = 1 and terminates if finding the block
function blkDetected = scan_in_Q1(s, blkDetected)
    angle = 0; %set initial angle;
        while (blkDetected == 0 && angle < 105) %rotate 105d in total 
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
            blkDetected = blkScanning(distBottom,distFront); %check for the block
        end

end


