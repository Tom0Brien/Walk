%% Load in data
data = importdata('data.mat');
opt_joint_angles = data.opt_joint_angles;
disp(opt_joint_angles)

TIME_STEP = data.p.Ts*1000;
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

%% Enable devices
% get and enable accelerometer
accelerometer = wb_robot_get_device('accelerometer');
wb_accelerometer_enable(accelerometer,TIME_STEP);
% get and enable gyro
gyro = wb_robot_get_device('gyroscope');
wb_gyro_enable(gyro,TIME_STEP);
% get and enable accelerometer with no noise
accelerometer_no_noise = wb_robot_get_device('accelerometer_no_noise');
wb_accelerometer_enable(accelerometer_no_noise,TIME_STEP);
% get and enable gyro with no noise
gyro_no_noise = wb_robot_get_device('gyroscope_no_noise');
wb_gyro_enable(gyro_no_noise,TIME_STEP);
%% Initialize arrays to store measurements
accelerometer_measurements = [];
gyroscope_measurements = [];
accelerometer_measurements_no_noise = [];
gyroscope_measurements_no_noise = [];
position = [];
velocity = [];
com_position = [];
orientation = [];
Htw = [];

init_pos = false;

% Get length of servo targets
sim_time = size(opt_joint_angles,2);
j = 1;
time = 0;
wait_time = 1;
walk_time = data.p.walk_time;
max_walk_time = walk_time + wait_time;

% supervisor
robot_node = wb_supervisor_node_get_self();

iteration = string(data.p.iteration);

% filename = convertStringsToChars(iteration)+"_iteration.MOV";
filename = 'walk.MOV';
%wait until ready to record
while(wb_supervisor_movie_is_ready() == 0)
end
wb_supervisor_movie_start_recording(convertStringsToChars(filename),1280,720,1,100,1,true);
while wb_robot_step(TIME_STEP) ~= -1
  time = time + TIME_STEP/1000
  if(time < wait_time & init_pos == true)
    disp(time)
    time = time + TIME_STEP/1000;
  else
    % Reset servo targets to first full step
    if(j == sim_time)
      j = floor(sim_time/3) + 1;
    end 
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
    init_pos = true;

    % COLLECT DATA
    accelerometer_measurements = [accelerometer_measurements; wb_accelerometer_get_values(accelerometer)];
    gyroscope_measurements = [gyroscope_measurements; wb_gyro_get_values(gyro)];
    accelerometer_measurements_no_noise = [accelerometer_measurements_no_noise; wb_accelerometer_get_values(accelerometer_no_noise)];
    gyroscope_measurements_no_noise = [gyroscope_measurements_no_noise; wb_gyro_get_values(gyro_no_noise)];
    orientation = [orientation; wb_supervisor_node_get_orientation(robot_node)];
    position = [position; wb_supervisor_node_get_position(robot_node)];
    velocity = [velocity; wb_supervisor_node_get_velocity(robot_node)];
    com_position = [com_position; wb_supervisor_node_get_center_of_mass(robot_node)];
    Htw = [Htw; [wb_supervisor_node_get_orientation(robot_node),wb_supervisor_node_get_position(robot_node).';[0,0,0,1]]];
    %% Store data in text file
    writematrix(accelerometer_measurements,'accelerometer_measurements');
    writematrix(gyroscope_measurements,'gyroscope_measurements');
    writematrix(accelerometer_measurements_no_noise,'accelerometer_measurements_no_noise');
    writematrix(gyroscope_measurements_no_noise,'gyroscope_measurements_no_noise');
    writematrix(orientation,'orientation');
    writematrix(position,'position');
    writematrix(Htw,'Htw');
    if(time > max_walk_time)
      save('result.mat','position','velocity','com_position')
      disp("Distance Traveled (x,y,z) = ")
      disp(position(end,:))
      wb_supervisor_movie_stop_recording();
      pause(8);
      wb_supervisor_simulation_quit(wb_supervisor_movie_failed());
      break;
    end
  end
end
