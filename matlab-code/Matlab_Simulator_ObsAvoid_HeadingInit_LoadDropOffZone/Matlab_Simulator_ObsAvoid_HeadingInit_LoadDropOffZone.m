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

%% Simulator or BT
sim = 1;
if sim
    % Initialize tcp server to read and respond to algorithm commands
    [s_cmd, s_rply] = tcp_setup();
    fopen(s_cmd);
    %fopen(s_rply);
else
    % connect to Bluetooth
    btInfo = instrhwinfo('Bluetooth','ROB6')
    s_cmd = Bluetooth(btInfo.RemoteID, 1);
    fopen(s_cmd);
    s_rply = s_cmd;
end

%% Robot Parameters
u = [0,0,0,0,0,0];  % Ultrasonic measurements % Updated
uTemp = [0,0,0,0,0,0]; % Updated
pos = [0,0,0];  % Position (x,y,rotation)
stepCount = 0;
forwardCount = 0;
leftRef = 0;
rightRef = 0;
count = 0;
stepbackcount = 0;
Icount = 1; % Initialization activation count
escape = 0; % Parameter for checking 4 way
i_c = 0;
i_done = 0;

% Probability Maxtrix's Size
row_n = size(p, 1); 
col_n = size(p, 2);

% Fixed path info, Each element consist of:
% Block Number / Heading (loading) / Destination % (loading, just for reference) / Heading (unloading) / Destination (unloading)
% When used in the function, use variable(index).split() and get an array of strings
all_path = ["1-180-Self-270-25"; "2-180-1-180-1"; "3-180-2-90-4"; "4-180-2-270-12"; "5----"; "6-270-14-90-Self"; "7----"; "8-270-16-90-Self"; "9-90-1-270-25"; "10-90-2-180-9"; "11----"; "12-90-4-0-14"; "13-180-12-0-14"; "14-180-12-90-6"; "15-180-12-0-16"; "16-180-12-90-8"; "17-90-9-270-25"; "18----"; "19-270-27-90-Self"; "20----"; "21----"; "22-270-30-90-14"; "23----"; "24-90-16-270-32"; "25-90-9-0-27"; "26-180-25-0-27"; "27-180-25-90-19"; "28-180-25-180-27"; "29-180-25-180-27"; "30-180-25-90-14"; "31----"; "32-90-16-270-Self"];

% Adjustable Parameters
speed = 2.4;
heading = 90;
load_unload = 0; % Loading = 0, Unloading = 1
unloading = 19; % Specify unloading zone
threshold = 0.1;
th_sum = 0.2;

%% Adjust all_path for unloading zone
all_path = path_adjust(all_path, unloading); 

%% Main Loop
while 1
	%% Take Measurements
    for ct = 1:6
       cmdstring = [strcat('u',num2str(ct)) newline];
       u(ct) = tcpclient_write(cmdstring, s_cmd, s_rply);
    end
    % cmdstring = ['ua' newline];
    % u = tcpclient_write(cmdstring, s_cmd, s_rply);
    ir(stepCount+1) = tcpclient_write(['i1' newline], s_cmd, s_rply);   
    comp = tcpclient_write(['c1' newline], s_cmd, s_rply); 
    
    %% Display Values
    if (ir(stepCount+1) == 1)
        fprintf('IR Sensor: %dblack\n', ir);
    else 
        fprintf('IR Sensor: %dwhite\n', ir);   
    end
    fprintf('Compass reading: %d\n',comp);
    disp('Heading: ' + string(heading))
    
    %% Sensor values 
    distFront = u(1); % Updated
    % disp(distFront)
    distL45 = u(2); % Updated
    distLeft = u(3); % Updated
    distRight = u(4); % Updated    
    distR45 = u(5); % Updated
    distBack = u(6); % Updated

    %% Update Probability
    p = sense_r(bw, M, p, ir(stepCount+1));
    figure(pfig)
    imagesc(p);
    title(['step: ' num2str(stepCount)]);
    
    %% Finding location where the probability > threshold
    % Just for checkin    
    % high_p = high_probability(row_n, col_n, threshold, p);  
    
    %% Threshold Probability value to start using fixed path
    % Using max value for now
    [p_max,I] = max(p(:));
    [p_row, p_col] = ind2sub(size(p),I);
    disp_pmax = ['Max Probability: ' + string(p_max)];
    disp(disp_pmax)
    % high_p_max = max(high_p);
    % disp('Max Sum Probability: ' + string(high_p_max))    
    %pause;
    
    %% Check if the robot is at 4 way (Sam)
    % escape = check4Way(distFront, distLeft, distRight, distBack, heading, s_cmd, s_rply);
    
    %% If the robot is at 4 way, drive straight and exit the 4 way
    if escape == 1
        % disp('Escape 4 way')
        for i = 1:4
        cmdstring = [strcat('d1-',num2str(speed)) newline];           
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        % disp("Move foward to escape 4 way")
        % call move function to update probability
        p = move(p, M, heading);
        angle_correction(u, distLeft, distRight, uTemp, s_cmd, s_rply);
        end
    end
    
    %% Check if the robot is at 4 way (Ryan)
    % 0 and 180 degree, front and rear has the same range of values
    
    if ( ( 21 <= distFront && distFront <= 33) && ( 21 <= distBack && distBack <= 33) )
        if ( ( 13 <= distLeft && distLeft <= 17) && ( 25 <= distRight && distRight <= 29) )
            heading = 0;
            i_c = 1;
            if i_done == 0
                p = ones(dim2,dim1)*(1/n);
                i_done = 1;
                %% Turn to 180 direction whenever its in the 4 way
                heading = turn_2_next_block(s_cmd, s_rply, heading, 180);
            end
            disp('4 Way Heading Update to ' + string(heading))
        elseif ( ( 13 <= distRight && distRight <= 17) && ( 25 <= distLeft && distLeft <= 29) )
            heading = 180;
            i_c = 1;
            if i_done == 0
                p = ones(dim2,dim1)*(1/n);
                i_done = 1;
                %% Turn to 180 direction whenever its in the 4 way
                heading = turn_2_next_block(s_cmd, s_rply, heading, 180);
            end
            disp('4 Way Heading Update to ' + string(heading))
        end
    % 90 and 270 degree, sides has the same range of values
    elseif ( ( 25 <= distLeft && distLeft <= 29) && ( 25 <= distRight && distRight <= 29) )
        if ( ( 9 <= distFront && distFront <= 21) && ( 21 <= distBack && distBack <= 33) )
            heading = 90;
            i_c = 1;
            if i_done == 0
                p = ones(dim2,dim1)*(1/n);
                i_done = 1;
                %% Turn to 180 direction whenever its in the 4 way
                heading = turn_2_next_block(s_cmd, s_rply, heading, 180);
            end
            disp('4 Way Heading Update to ' + string(heading))
        elseif ( ( 9 <= distBack && distBack <= 21) && ( 21 <= distFront && distFront <= 33) )
            heading = 270; 
            i_c = 1;
            if i_done == 0
                p = ones(dim2,dim1)*(1/n);
                i_done = 1;
                %% Turn to 180 direction whenever its in the 4 way
                heading = turn_2_next_block(s_cmd, s_rply, heading, 180);
            end
            disp('4 Way Heading Update to ' + string(heading))
        end
    end
    
    % Reset probability when the robot initialize its heading
   
    %% Check if the robot is in block 28 to correction heading
    % heading = initialize(distFront, distRight, distLeft, distBack, heading);
    
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
            disp('Heading to Loading Zone')
            %% If in the loading zone already
            if sum(find( cur_block == ["1", "2", "9", "10"] > 0)) > 0
                if distFront >= 4
                    [p, heading] = forward_load_unload(distLeft, distRight, distL45, distR45, heading, speed, s_cmd, s_rply, p, M, path_info_a, load_unload);
                    %% Angle correction logic
                    angle_correction(u, distLeft, distRight, uTemp, s_cmd, s_rply);
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
                    heading = turn_2_next_block(s_cmd, s_rply, heading, heading_change);
                end
            
            %% If in the right vertical corridor, exit the corridor when the robot is in block 16
            elseif cur_block == "16"
                if (heading == 90 && distFront <= 17) || (heading == 270 && distFront <= 29)
                    dest_heading = 180;
                    heading_change = (dest_heading - heading);
                    %% Turning to the next block
                    heading = turn_2_next_block(s_cmd, s_rply, heading, heading_change);
                end    
                
            %% In other blocks
            else 
                %% Move robot:
                % Updates the heading only when the robot is close to the wall
                [p, heading] = obstacle_avoidance_fp(distLeft, distRight, distFront, distL45, distR45, heading, speed, s_cmd, s_rply, u, uTemp, p, M, path_info_a, load_unload);
            end

        %% Going to Unloading Zone
        else
            disp('Heading to Unloading Zone')
            %% For Block 27
            % Block 19 is a special unloadig zone since the robot needs to turn
            % up with the front distance ~96 cm at block 27
            dest_heading = str2num(path_info_a(4));
            if cur_block_n == 27 && unloading == 19 && heading ~= 90
                disp('Front Distance: ' + string(distFront))
                if ( 26 <= distFront && distFront <= 28) || ( 38 <= distFront && distFront <= 40)
                    heading_change = (dest_heading - heading);
                    %% Turning to the next block
                    heading = turn_2_next_block(s_cmd, s_rply, heading, heading_change);
                else
                    [p, heading] = forward_load_unload(distLeft, distRight, distL45, distR45, heading, speed, s_cmd, s_rply, p, M, path_info_a, load_unload);
                    %% Angle correction logic 
                    angle_correction(u, distLeft, distRight, uTemp, s_cmd, s_rply);
                end
                
            %% Adjust the heading when the robot is in the bottom corridor & missed block 19
            elseif sum(find( cur_block == ["26", "28", "29"] > 0)) > 0 && unloading == 19 && heading ~= dest_heading
                heading_change = (dest_heading - heading);
                %% Turning to the next block
                heading = turn_2_next_block(s_cmd, s_rply, heading, heading_change);                
            
            %% In unloading zone, drive to the middle of the block 
            elseif cur_block_n == unloading
                if distFront >= 4
                    [p, heading] = forward_load_unload(distLeft, distRight, distL45, distR45, heading, speed, s_cmd, s_rply, p, M, path_info_a, load_unload);
                    %% Angle correction logic
                    angle_correction(u, distLeft, distRight, uTemp, s_cmd, s_rply);
                else
                    disp("-------------------------")
                    disp("I am at the unloading zone!")
                    disp("-------------------------")
                end
                
            %% In other blocks
            else
                %% Motor control when following Fixed Path & Not in Loading/Unloading:
                [p, heading] = obstacle_avoidance_fp(distLeft, distRight, distFront, distL45, distR45, heading, speed, s_cmd, s_rply, u, uTemp, p, M, path_info_a, load_unload);
            end
        end
                
    %% If the probability is not high enough, just do obstacle avoidance    
    else                
        %% Motor control original:
        % Obstacle Avoidance including Angle Correction
        [p, heading] = obstacle_avoidance(distLeft, distRight, distFront, distL45, distR45, heading, speed, s_cmd, s_rply, u, uTemp, p, M);       
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
% Example
% current_block = block_finder(row_n, col_n); 

cur_block = '';

% Row 1-4 
% Column A-H

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

%% Finding location where the probability > threshold
function high_p = high_probability(row_n, col_n, threshold, p)
% high_p = high_probability(row_n, col_n, threshold, p);
    % Just for checkin    
    % high_p = strings(); % An array of string to store the location of high probability
    high_p = zeros(1, 32);
    % hpc = 1; % For indexing the array
    block = 0;
    for i = 1: row_n
        for j = 1: col_n
            if p(i, j) > threshold
                % Summing any probability in the same block
                block = block_finder(i, j);
                block_n = str2num(block);
                high_p(block_n) = high_p(block_n) + p(i,j);
                % Storing the row column heading of a loction that has p > threshold
                % block = block_finder(i, j);
                % high_p(hpc) = strcat('Current block: ', block, '  Heading: ', string(heading), '  Probability: ', string(p(i, j)));                
                % keeping for future use
                % 'Row; ', string(i), '  Column: ', string(j), strcat('Current block: ', block, '  Heading: ', string(heading), '  Probability: ', string(p(i, j)))
                % hpc = hpc + 1;
            end
        end
    end  
end

%% Check if the robot is at 4 way (Sam)
function escape = check4Way(distFront, distLeft, distRight, distBack, heading, s_cmd, s_rply)
% disp('Checking 4 way')
% Check if all 4 direction has a huge distance
if (distFront > 10 && distLeft > 10 && distRight > 10 && distBack > 10)
    pause(1);
    % Turn Left and check again
    cmdstring = [strcat('r1-',num2str(90)) newline]; 
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    heading = mod((heading + 90),360); % Left turn;
    % disp("Turn left x1 to check 4 way again")
    if (distFront > 10 && distLeft > 10 && distRight > 10 && distBack > 10)
        % disp("At 4 way")
        escape = 1;
    else
        escape = 0;
    end
else
    escape = 0;
end
% Turn Right
cmdstring = [strcat('r1-',num2str(-90)) newline];
reply = tcpclient_write(cmdstring, s_cmd, s_rply);
heading = mod((heading - 90),360); %right turn;
% disp("Turn right x1 to move in original heading") 

end

%% Block 28 Initialization (Daniel)
function heading = initialize(distFront, distRight, distLeft, distBack, heading_0)
if (distFront >= 24 && distRight <= 5 && distLeft <= 5 && distBack >= 36)
    heading = 0;    
elseif (distFront <= 5 && distRight >= 36 && distLeft >= 24 && distBack <= 5)
    heading = 270;    
elseif (distFront >= 36 && distRight <= 5 && distLeft <= 5 && distBack >= 24)
    heading = 180;     
elseif (distFront <= 5 && distRight >= 24 && distLeft >= 36 && distBack <= 5)
    heading = 90;     
else
    heading = heading_0;
end  
disp("New heading: " + heading)
end

%% Obstacle Avoidance including Angle Correction
function [p, heading] = obstacle_avoidance(distLeft, distRight, distFront, distL45, distR45, heading, speed, s_cmd, s_rply, u, uTemp, p, M)
% 
%% Motor control original:
if (distFront >= 4)
    if ( distLeft > 1.3 && distRight > 1.3 )
        %% all clear, move forward. Every three steps count as one
        if ( distL45 > 3 && distR45 > 3 )
            cmdstring = [strcat('d1-',num2str(speed)) newline];           
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            disp("1 foward, w")
            %call move function to update probability
            p = move(p, M, heading);
        
        %% Adjustment
        else 
            cmdstring = [strcat('d1-',num2str(-(speed/4))) newline];           
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            %% Small Adjusment
            if ( distL45 > distR45 )
                cmdstring = [strcat('r1-',num2str(5)) newline];           
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                disp('1 towards left')
            else 
                cmdstring = [strcat('r1-',num2str(-5)) newline];           
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                disp('1 towards right')
            end
            disp("Stepback")
        end
        
        %% Angle correction logic
        rightRef = distRight; % Updated
        leftRef = distLeft; % Updated
        % Take side measurements for correction
        for ct = 3:4 % Updated
            cmdstring = [strcat('u', num2str(ct)) newline];
            uTemp(ct) = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
        leftTemp = uTemp(3); % Updated
        rightTemp = uTemp(4); % Updated
        
        %% Use the side that has smaller distance to compare
        if ( rightTemp <= leftTemp )
            if ( rightTemp > rightRef )
                %robot drifting towards left, correct towards
                %right by 5 degrees
                cmdstring = [strcat('r1-',num2str(-5)) newline]; 
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                disp("Correction towards right, d")
            elseif ( rightTemp < rightRef )
                %robot drifting towards right, correct towards
                %left by 5 degrees
                cmdstring = [strcat('r1-',num2str(5)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                disp("Correction towards left, a")
            else
                disp("No correction")
            end
        else % Left < right
            if ( leftTemp > leftRef )
                cmdstring = [strcat('r1-',num2str(5)) newline]; 
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                disp("Correction towards left")
            elseif (leftTemp < leftRef)
                cmdstring = [strcat('r1-',num2str(-5)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                disp("Correction towards right") 
            else 
                disp("No correction")
            end
        end
        %end of angle correction logic
    else % distLeft < 1 || distRight < 1 The front is clear, but one side is blocked
        if ( distLeft > distRight )
            cmdstring = [strcat('r1-',num2str(90)) newline]; 
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            disp("2 left, a")
            heading = mod((heading + 90),360); %left turn;
        else % disRight > disLeft
            cmdstring = [strcat('r1-',num2str(-90)) newline]; 
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            disp("3 right, d")
            heading = mod((heading - 90),360); %right turn
        end
    end
    
%% Front is bloked, turn to the side with larger room
else % distFront < 4. The front is blocked.
    if ( distLeft > distRight )
        %turn to the side with larger clearance
        cmdstring = [strcat('r1-',num2str(90)) newline];  
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp("4 left, a")
        heading = mod((heading + 90),360); %left turn
    else % distRight > distLeft
        cmdstring = [strcat('r1-',num2str(-90)) newline]; 
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp("5 right, d")
        heading = mod((heading - 90),360); %right turn
    end
end
% pause;
end

%% Angle correction logic
function angle_correction(u, distLeft, distRight, uTemp, s_cmd, s_rply)
% [cmdstring, reply] = 
rightRef = distRight; % Updated
leftRef = distLeft; % Updated
% take side measurements for correction
for ct = 3:4 % Updated
    cmdstring = [strcat('u',num2str(ct)) newline];
    uTemp(ct) = tcpclient_write(cmdstring, s_cmd, s_rply);
end
leftTemp = uTemp(3); % Updated
rightTemp = uTemp(4); % Updated

%% Use the side that has smaller distance to compare
if ( rightTemp <= leftTemp )                        
    if ( rightTemp > rightRef )
        %robot drifting towards left, correct towards
        %right by 2 degrees
        cmdstring = [strcat('r1-',num2str(-5)) newline]; 
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp("Correction towards right, d")
    elseif ( rightTemp < rightRef )
      %robot drifting towards right, correct towards
      %left by 2 degrees
        cmdstring = [strcat('r1-',num2str(5)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp("Correction towards left, a")
    else
        disp("No correction")
    end
else % left < right
    if ( leftTemp > leftRef )
        cmdstring = [strcat('r1-',num2str(5)) newline]; 
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp("Correction towards left")
    elseif ( leftTemp < leftRef )
        cmdstring = [strcat('r1-',num2str(-5)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp("Correction towards right") 
    else 
        disp("No correction")
    end
end
% end of angle correction logic
end

%% Obstacle Avoidance when following Fixed Path & Not in Loading/Unloading:
function [p, heading] = obstacle_avoidance_fp(distLeft, distRight, distFront, distL45, distR45, heading, speed, s_cmd, s_rply, u, uTemp, p, M, path_info_a, load_unload)
%  
if (distFront >= 4)
    if ( distLeft > 1.3 && distRight > 1.3)
        %% All clear, move forward
        if ( distL45 > 3 && distR45 > 3 )
            cmdstring = [strcat('d1-',num2str(speed)) newline];           
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            disp("1 foward, w")
            %call move function to update probability
            p = move(p, M, heading);
        
        %% Adjustment
        else 
            cmdstring = [strcat('d1-',num2str(-(speed/4))) newline];           
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            %% Small Adjusment
            if ( distL45 > distR45 )
                cmdstring = [strcat('r1-',num2str(5)) newline];           
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                disp('1 towards left')
            else 
                cmdstring = [strcat('r1-',num2str(-5)) newline];           
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                disp('1 towards right')
            end
            disp("Stepback")
        end
        
        %% Angle correction logic
        rightRef = distRight; % Updated
        leftRef = distLeft; % Updated
        % Take side measurements for correction
        for ct = 3:4 % Updated
            cmdstring = [strcat('u',num2str(ct)) newline];
            uTemp(ct) = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
        leftTemp = uTemp(3); % Updated
        rightTemp = uTemp(4); % Updated
        
        %% Use the side that has smaller distance to compare
        if ( rightTemp <= leftTemp )
            if ( rightTemp > rightRef )
                %robot drifting towards left, correct towards
                %right by 5 degrees
                cmdstring = [strcat('r1-',num2str(-5)) newline]; 
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                disp("Correction towards right, d")
            elseif ( rightTemp < rightRef )
                %robot drifting towards right, correct towards
                %left by 5 degrees
                cmdstring = [strcat('r1-',num2str(5)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                disp("Correction towards left, a")
            else
                disp("No correction")
            end
        else % Left < right
            if ( leftTemp > leftRef )
                cmdstring = [strcat('r1-',num2str(5)) newline]; 
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                disp("Correction towards left")
            elseif (leftTemp < leftRef)
                cmdstring = [strcat('r1-',num2str(-5)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                disp("Correction towards right") 
            else 
                disp("No correction")
            end
        end
        %end of angle correction logic
    else % distLeft < 1 || distRight < 1 The front is clear, but one side is blocked
        if ( distLeft > distRight )
            cmdstring = [strcat('r1-',num2str(90)) newline]; 
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            disp("2 left, a")
            heading = mod((heading + 90),360); %left turn;
        else % disRight > disLeft
            cmdstring = [strcat('r1-',num2str(-90)) newline]; 
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            disp("3 right, d")
            heading = mod((heading - 90),360); %right turn
        end
    end
   
%% Front is bloked, Adjust Robot Heading to dest_heading
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
    heading = turn_2_next_block(s_cmd, s_rply, heading, heading_change);
end
% pause;
end

%% Turn to the next block
function heading = turn_2_next_block(s_cmd, s_rply, heading_0, heading_change)
%
heading = heading_0;
if heading_change == 270 || heading_change == -90
    % Turn right x 1
    cmdstring = [strcat('r1-',num2str(-90)) newline]; 
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    heading = mod((heading - 90),360); %right turn;
    disp("Turn right x1")                        
elseif heading_change == -180 || heading_change == 180
    % Turn right x 2
    cmdstring = [strcat('r1-',num2str(-180)) newline]; 
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    heading = mod((heading - 180),360); %Turn Around;
    disp("Turn 180")
elseif heading_change == 90 || heading_change == -270
    % Turn left x 1
    cmdstring = [strcat('r1-',num2str(90)) newline]; 
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    heading = mod((heading + 90),360); %left turn;
    disp("Turn left x1")                    
else % Heading same and destination heading, no change
end
end

%% Move forward in loading/unloading
function [p, heading] = forward_load_unload(distLeft, distRight, distL45, distR45, heading, speed, s_cmd, s_rply, p, M, path_info_a, load_unload)
%  
%% All clear, move forward
if ( distLeft > 1.3 && distRight > 1.3 && distL45 > 3 && distR45 > 3 )
    %all clear, move forward. Every  three steps count as one
    %movement
    cmdstring = [strcat('d1-',num2str(speed)) newline];           
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    disp("1 foward, w")
    %call move function to update probability
    p = move(p, M, heading);

%% Adjustment
else 
    cmdstring = [strcat('d1-',num2str(-(speed/4))) newline];           
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    %% Small Adjusment
    if (distL45 > distR45)
        cmdstring = [strcat('r1-',num2str(5)) newline];           
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('1 towards left')
    else 
        cmdstring = [strcat('r1-',num2str(-5)) newline];           
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('1 towards right')
    end
    disp("Stepback")
end
end


