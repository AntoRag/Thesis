clear all
close all
clc
%% Importing data
% To import data the following function is implemented. It will read the 
% .csv files to get postion and velocities for the arm and the postion
% of the wheels of the robot. The joint referred to the arm are:
%   - elbow
%   - forearm_roll
%   - gripper
%   - left_finger
%   - pan
%   - right_finger
%   - shoulder
%   - tilt
%   - waist
%   - wrist_angle
%   - wrist_rotate
% Where pan and tilt are the servos for the camera turret and right and
% left finger are the servos of the end effector. The ones responsible for
% the motion of the base are:
%   - wheel_left_joint
%   - wheel_right_joint

% data_arm =
% readmatrix('C:\Thesis\Matlab\Data\data_saved_arm.csv','OutputType','double');
data_arm = readmatrix('/home/antonio/thesis/Matlab/Data/data_saved_arm.csv','OutputType','double');

%% Reordering vector
time = data_arm(1:end,1);
arm_angles_pos = [data_arm(:,10) data_arm(:,8) data_arm(:,2) data_arm(:,3) data_arm(:,11) data_arm(:,12)];
gripper_angles_pos = [data_arm(:,5) data_arm(:,7)];
camera_angles_pos = [data_arm(:,6) data_arm(:,9)];
arm_angles_vel = [data_arm(:,21) data_arm(:,19) data_arm(:,13) data_arm(:,14) data_arm(:,22) data_arm(:,23)];
gripper_angles_vel = [data_arm(:,16) data_arm(:,18)];
camera_angles_vel = [data_arm(:,17) data_arm(:,20)];

%% Joint ordered
%% Arm angles
% - Waist
% - Shoulder
% - Elbow
% - Forearm_roll
% - Wrist angle
% - Wrist rotate
%% Gripper angles
% - Left finger
% - Right finger
%% Camera angles
% - Pan
% - Tilt

%% Verifying the results via direct kinematic
p = zeros(length(data_arm),3);
RPY = zeros(length(data_arm),3);

for i = 1:length(arm_angles_pos)
    [M,position,rpy] = DirectKinematic(arm_angles_pos(i,:));
    p(i,:) = position';
    RPY(i,:) = [rpy];
end
initial_position = [p(1,1),p(1,2),p(1,3)];
final_position = [p(end,1),p(end,2),p(end,3)];

%% Plotting the results obtained for the arm
% 
% joint_arm = string(["Elbow" ,"Forearm roll", "Gripper" ,"Left finger", "Pan", ...
%     "Right finger", "Shoulder", "Tilt", "Waist", "Wrist angle", "Wrist rotate"]);
joint_arm = string(["Waist","Shoulder","Elbow","Forearm roll","Wrist angle",...
    "Wrist rotate","Left finger","Right finger","Pan","Tilt"]);
for i = 1:6
    figure(i)
    subplot 211
    hold on
    title(string(joint_arm(i)) +' positions')
    plot(time,[arm_angles_pos(1:end,i)])
    grid on, zoom on
    xlabel('time [s]'), ylabel('Radiants [rad]')

    subplot 212
    hold on
    title(string(joint_arm(i))+ ' velocities')
    plot([arm_angles_pos(1:end,1)],[arm_angles_vel(1:end,i)])
    grid on, zoom on
    xlabel('time [s]'), ylabel('Radians [rad]')
end

%% Plotting the trajectory of the end effector

figure
grid, hold on
plot3(p(:,1),p(:,2),p(:,3))             % trajectory
xlabel('$x [m] $','interpreter','latex','fontsize',20)
ylabel('$y[m] $','interpreter','latex','fontsize',20)
zlabel('$z[m] $','interpreter','latex','fontsize',20)
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])
plot3(initial_position(1,1),initial_position(1,2),initial_position(1,3),'-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF')
plot3(final_position(end,1),final_position(end,2),final_position(end,3),'-o','Color','r','MarkerSize',10,'MarkerFaceColor','#D9FFFF')
view(20,50)


