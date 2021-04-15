function output = initialConditionsLeft
l_hip_yaw = 0.02923973;
r_hip_yaw = 0.02923973;

l_hip_roll = 0.06261409;
r_hip_roll = -0.06261409;

l_ankle_roll = -0.06909663;
r_ankle_roll = 0.06909663;

l_hip_pitch = 0.207138;
r_hip_pitch = 0.207138;

l_knee_pitch = 0.2945004;
r_knee_pitch = 0.2945004;

l_ankle_pitch = -0.2623748;
r_ankle_pitch = -0.2623748;


neck_yaw = 0;
output = [
            l_ankle_roll;
            l_ankle_pitch; 
            l_knee_pitch;
            l_hip_pitch;
            l_hip_roll;
            l_hip_yaw;
            r_hip_yaw;
            r_hip_roll;
            r_hip_pitch;
            r_knee_pitch;            
            r_ankle_pitch; 
            r_ankle_roll;        
            neck_yaw; 
];

end
