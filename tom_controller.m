% MATLAB controller for Webots
% File:          tom_controller.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 10;

% get and enable devices, e.g.:
wb_robot_init();
n_devices = wb_robot_get_number_of_devices();
for i=1:n_devices
   disp(wb_device_get_name(wb_robot_get_device_by_index(i - 1)));
end

right_ankle_pitch = wb_robot_get_device('right_ankle_pitch');
right_knee_pitch = wb_robot_get_device('right_knee_pitch');
right_hip_pitch = wb_robot_get_device('right_hip_pitch');
left_hip_pitch = wb_robot_get_device('left_hip_pitch');
left_knee_pitch = wb_robot_get_device('left_knee_pitch');
left_hip_yaw = wb_robot_get_device('left_hip_yaw');
left_hip_roll = wb_robot_get_device('left_hip_roll');
left_ankle_roll = wb_robot_get_device('left_ankle_roll');
right_hip_yaw = wb_robot_get_device('right_hip_yaw');
right_hip_roll = wb_robot_get_device('right_hip_roll');
right_ankle_roll = wb_robot_get_device('right_ankle_roll');
right_shoulder_pitch = wb_robot_get_device('right_shoulder_pitch');
right_shoulder_roll = wb_robot_get_device('right_shoulder_roll');
right_elbow_pitch = wb_robot_get_device('right_elbow_pitch');
left_shoulder_pitch = wb_robot_get_device('left_shoulder_pitch');
left_shoulder_roll = wb_robot_get_device('left_shoulder_roll');
left_elbow_pitch = wb_robot_get_device('left_elbow_pitch');
left_ankle_pitch = wb_robot_get_device('left_ankle_pitch');
left_knee_pitch = wb_robot_get_device('left_knee_pitch');
left_hip_pitch = wb_robot_get_device('left_hip_pitch');
right_ankle_pitch = wb_robot_get_device('right_ankle_pitch');
right_knee_pitch = wb_robot_get_device('right_knee_pitch');
right_hip_pitch = wb_robot_get_device('right_hip_pitch');
neck_yaw = wb_robot_get_device('neck_yaw');
head_pitch = wb_robot_get_device('head_pitch');


wb_motor_set_position(left_hip_yaw, 0.02923973);
wb_motor_set_position(left_hip_roll, 0.06261409);
wb_motor_set_position(left_ankle_roll, -0.06909663);
wb_motor_set_position(right_hip_yaw, -0.02923973);
wb_motor_set_position(right_hip_roll, -0.06261409);
wb_motor_set_position(right_ankle_roll, 0.06909663);
wb_motor_set_position(right_shoulder_pitch, 1.963495);
wb_motor_set_position(right_shoulder_roll, -0.1239184);
wb_motor_set_position(right_elbow_pitch, -2.443461);
wb_motor_set_position(left_shoulder_pitch, 1.963495);
wb_motor_set_position(left_shoulder_roll, 0.1239184);
wb_motor_set_position(left_elbow_pitch, -2.443461);
wb_motor_set_position(left_ankle_pitch, -0.2623748);
wb_motor_set_position(left_knee_pitch, 0.2945004);
wb_motor_set_position(left_hip_pitch, -0.207138);
wb_motor_set_position(right_ankle_pitch, -0.2393594);
wb_motor_set_position(right_knee_pitch, 0.2561415);
wb_motor_set_position(right_hip_pitch, -0.2040693);

%knee_position_sensor = wb_motor_get_position_sensor(right_knee_pitch);
%wb_position_sensor_enable(knee_position_sensor, TIME_STEP);
%ankle_position_sensor = wb_motor_get_position_sensor(right_ankle_pitch);
%wb_position_sensor_enable(ankle_position_sensor, TIME_STEP);
%hip_position_sensor = wb_motor_get_position_sensor(right_hip_pitch);
%wb_position_sensor_enable(hip_position_sensor, TIME_STEP);

opt_joint_angles = importdata('servo_positions.mat');

sim_time = length(opt_joint_angles);   
j = 1;
time = 0;
wb_motor_set_velocity(right_ankle_pitch,1);
wb_motor_set_velocity(right_knee_pitch,0.25);
wb_motor_set_velocity(right_hip_pitch,0.25);
wb_motor_set_velocity(left_knee_pitch,0.25);
while wb_robot_step(TIME_STEP) ~= -1
time = time + TIME_STEP;
  if(j < sim_time - 1 & time/100 > j) %& (wb_motor_get_target_position(right_knee_pitch)-wb_position_sensor_get_value(knee_position_sensor)) < 0.05)% & (wb_motor_get_target_position(right_ankle_pitch)-wb_position_sensor_get_value(ankle_position_sensor)) < 0.05 & (wb_motor_get_target_position(right_hip_pitch)-wb_position_sensor_get_value(hip_position_sensor)) < 0.05)
    wb_motor_set_position(left_hip_yaw,opt_joint_angles(1,j));
    wb_motor_set_position(left_hip_roll,opt_joint_angles(2,j));
    wb_motor_set_position(left_hip_pitch,opt_joint_angles(3,j));
    wb_motor_set_position(left_knee_pitch,opt_joint_angles(4,j));
    wb_motor_set_position(left_ankle_pitch,opt_joint_angles(5,j));
    wb_motor_set_position(left_ankle_roll,opt_joint_angles(6,j));
    wb_motor_set_position(left_shoulder_pitch,opt_joint_angles(7,j));
    wb_motor_set_position(left_shoulder_roll,opt_joint_angles(8,j));
    wb_motor_set_position(left_elbow_pitch,opt_joint_angles(9,j));
    wb_motor_set_position(neck_yaw,opt_joint_angles(10,j));
    wb_motor_set_position(head_pitch,opt_joint_angles(11,j));
    wb_motor_set_position(right_hip_yaw,opt_joint_angles(12,j));
    wb_motor_set_position(right_hip_roll,opt_joint_angles(13,j));
    wb_motor_set_position(right_hip_pitch,opt_joint_angles(14,j));
    wb_motor_set_position(right_knee_pitch,opt_joint_angles(15,j));
    wb_motor_set_position(right_ankle_pitch,opt_joint_angles(16,j));
    wb_motor_set_position(right_ankle_roll,opt_joint_angles(17,j));
    wb_motor_set_position(right_shoulder_pitch,opt_joint_angles(18,j));
    wb_motor_set_position(right_shoulder_roll,opt_joint_angles(19,j));
    wb_motor_set_position(right_elbow_pitch ,opt_joint_angles(20,j));
      j = j + 1;
      disp(time);
  end
  drawnow;

end
