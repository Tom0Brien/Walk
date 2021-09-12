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
left_hip_roll = wb_robot_get_device('left_hip_roll [hip]');
left_ankle_roll = wb_robot_get_device('left_ankle_roll');
right_hip_yaw = wb_robot_get_device('right_hip_yaw');
right_hip_roll = wb_robot_get_device('right_hip_roll [hip]');
right_ankle_roll = wb_robot_get_device('right_ankle_roll');
right_shoulder_pitch = wb_robot_get_device('right_shoulder_pitch [shoulder]');
right_shoulder_roll = wb_robot_get_device('right_shoulder_roll');
right_elbow_pitch = wb_robot_get_device('right_elbow_pitch');
left_shoulder_pitch = wb_robot_get_device('left_shoulder_pitch [shoulder]');
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

velocity = 5.76;
wb_motor_set_velocity(left_hip_yaw, velocity);
wb_motor_set_velocity(left_hip_roll, velocity);
wb_motor_set_velocity(left_ankle_roll, velocity);
wb_motor_set_velocity(right_hip_yaw, velocity);
wb_motor_set_velocity(right_hip_roll, velocity);
wb_motor_set_velocity(right_ankle_roll, velocity);
wb_motor_set_velocity(right_shoulder_pitch, velocity);
wb_motor_set_velocity(right_shoulder_roll, velocity);
wb_motor_set_velocity(right_elbow_pitch, velocity);
wb_motor_set_velocity(left_shoulder_pitch, velocity);
wb_motor_set_velocity(left_shoulder_roll, velocity);
wb_motor_set_velocity(left_elbow_pitch, velocity);
wb_motor_set_velocity(left_ankle_pitch, velocity);
wb_motor_set_velocity(left_knee_pitch, velocity);
wb_motor_set_velocity(left_hip_pitch, velocity);
wb_motor_set_velocity(right_ankle_pitch, velocity);
wb_motor_set_velocity(right_knee_pitch, velocity);
wb_motor_set_velocity(right_hip_pitch, velocity);

% Set true if initializing
init_conditions = false;

% Import servo targets
opt_joint_angles = importdata('servo_positions.mat');

% Get length of servo targets
sim_time = size(opt_joint_angles,2)
j = 1;
time = 0;
halfstep = true;
stop = false;
while wb_robot_step(TIME_STEP) ~= -1
time = time + TIME_STEP;
  % Reset servo targets to first full step
  if(j == sim_time)
    j = sim_time/3 + 1;
    time = j * 100;
  end 
  if(stop == false)
    wb_motor_set_position(left_hip_yaw,opt_joint_angles(1,j));
    wb_motor_set_position(left_hip_roll,opt_joint_angles(2,j));
    wb_motor_set_position(left_hip_pitch,opt_joint_angles(3,j));
    wb_motor_set_position(left_knee_pitch,opt_joint_angles(4,j));
    wb_motor_set_position(left_ankle_pitch,opt_joint_angles(5,j));
    wb_motor_set_position(left_ankle_roll,opt_joint_angles(6,j));
    wb_motor_set_position(left_shoulder_pitch,opt_joint_angles(7,j));
    wb_motor_set_position(left_shoulder_roll,opt_joint_angles(8,j));
    wb_motor_set_position(left_elbow_pitch,opt_joint_angles(9,j));
    %wb_motor_set_position(neck_yaw,opt_joint_angles(10,j));
    %wb_motor_set_position(head_pitch,opt_joint_angles(11,j));
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
    if(init_conditions == true)
      stop = true;
    end
  end
end
