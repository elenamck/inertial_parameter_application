init_pos = zeros(3,3);
init_ori = zeros(3,3,3);

aux_pos = zeros(3,3);
aux_ori = zeros(3,3,3);

%%%Object specific picks

pick_pos_before = zeros(3,3);
pick_ori_before = zeros(3,3,3);

pick_pos = zeros(3,3);
pick_ori = zeros(3,3,3);

pick_pos_above = zeros(3,3);
pick_ori_above = zeros(3,3,3);

carry_pos_1 = zeros(3,3);
carry_pos_2 = zeros(3,3);
carry_pos_3 = zeros(3,3);
carry_pos_4 = zeros(3,3);

carry_ori_1 = zeros(3,3,3);
carry_ori_2 = zeros(3,3,3);
carry_ori_3 = zeros(3,3,3);
carry_ori_4 = zeros(3,3,3);

place_pos_above = zeros(3,3);
place_ori_above = zeros(3,3);

place_pos = zeros(3,3);
place_ori = zeros(3,3,3);




%%% Cylinder

init_pos(1,:) = [0.52, -0.18, 0.59]
init_ori(:,:,1) = [ 0,-1, 0;
                   -1, 0, 0;
                    0, 0,-1];
init_vel = [0.0,0.0,0.0];
aux_pos(:,1) = [0.7,0.0,0.45];
aux_ori(:,:,1) = [0.52, -0.617,-0.01;
                 -0.43, 0.121,-0.9895;
                  0.74, 0.778,0.144];
aux_vel = [0.1,0.1,0.1];

%%%Object specific picks  
             
pick_pos_before(1,:) = [0.696665, -0.39134 ,0.251281]; 
pick_ori_before(:,:,1) = [0.787, -0.617,-0.01;
                      0.078, 0.121,-0.9895;
                      0.6121, 0.778,0.144];   
pick_vel_before = [0.05,0.05,0.05];                  

pick_pos(1,:) =   [0.695176 ,-0.396105 , 0.253126]; %%two times  ->pregrasp               
pick_ori(:,:,1) = [ 0.365173 ,  -0.840191 ,  -0.400909;
                   -0.00795357  ,  0.427819 ,   -0.90383;
                    0.930906 ,   0.333243   , 0.149545];
pick_vel = [0.0,0.0,0.0];                  

pick_ori_second(:,:,1) = [  0.539264  , -0.74728 , -0.388288;
                    -0.0219643  , 0.448437 , -0.893545;     %cyl ori
                     0.84185 ,  0.490385 ,  0.225412];

pick_pos_above(1,:) = [0.695176 ,-0.396105 , 0.253126];
pick_ori_above(:,:,1) = [ 0.66  , -0.67 , 0.335;
                  -0.23  , -0.6 , -0.758;
                   0.71 ,  0.42,  -0.558];

pick_vel_above = [0.05,0.05,0.05];
carry_pos_1(1,:) =[ 0.64 , -0.11, 0.403];%carry
carry_ori_1(:,:,1) = [  0.383 , -0.888 , 0.251;
                -0.62  , -0.449 , -0.643;
                -0.685 ,  0.0906996 , -0.723];
carry_vel = [0.1,0.1,0.1];
carry_pos_2(1,:) =[ 0.6,0.0,0.43]; %carry
carry_ori_2(:,:,1) =[ 0.9 , -0.4044, 0.131;
              -0.42  , -0.879 , 0.218;
               0.03,  -0.253 , -0.967];  
carry_pos_3(1,:) =[ 0.54,0.2,0.4] ; %carry
carry_ori_3(:,:,1) =[ 0.9 , -0.345, -0.28;
              -0.054  , -0.713 , 0.7;
               0.44,  -0.61 , -0.66];  %carry

carry_pos_4(1,:) =[ 0.51,0.24,0.407 ] ; %carry
carry_ori_4(:,:,1) = [ 0.465, -0.665, -0.584;
                0.252, -0.533 , 0.8;
                0.848,  -0.523 , -0.08];



place_pos_above(1,:) = [0.4758, 0.4375,0.303];
place_ori_above(:,:,1) = [0.38, -0.39,  -0.837;
                   0.519,-0.6588, 0.545;
                   -0.766, 0.64, -0.05];
place_vel_above = [0.01,0.01,0.01]
place_pos(1,:) = [0.4758, 0.4375,0.22];
place_ori(:,:,1) = [0.38, -0.39,  -0.837;
             0.519,-0.6588, 0.545;
            -0.766, 0.64, -0.05];
place_vel = [0.0,0.0,0.0];
waypoints = [init_pos(1,:)
             aux_pos(1,:)
             pick_pos_before(1,:)
             pick_pos(1,:)
             pick_pos(1,:)
             pick_pos_above(1,:)
             carry_pos_1(1,:)
             carry_pos_2(1,:)
             carry_pos_3(1,:)
             carry_pos_4(1,:)
             place_pos_above(1,:)
             place_pos(1,:)];    
orientation = zeros(3,3,12) ;  
orientation(:,:,1) = init_ori(:,:,1)
orientation(:,:,2) =  aux_ori(:,:,1)
orientation(:,:,3) =           pick_ori_before(:,:,1)
orientation(:,:,4) =           pick_ori(:,:,1)
orientation(:,:,5) =           pick_ori(:,:,1) 
orientation(:,:,6) =            pick_ori_above(:,:,1)
orientation(:,:,7) =         carry_ori_1(:,:,1)
orientation(:,:,8) =        carry_ori_2(:,:,1)
orientation(:,:,9) =        carry_ori_3(:,:,1)
orientation(:,:,10)=             carry_ori_4(:,:,1)
orientation(:,:,11)=        place_ori_above(:,:,1)
orientation(:,:,12)=        place_ori(:,:,1)
velocity =  [init_vel,
             aux_vel,
             pick_vel_before,
             pick_vel,
             pick_vel,
             pick_vel_above,
             carry_vel,
             carry_vel,
             carry_vel,
             carry_vel,
             place_vel_above,
             place_vel]  ;
velocity =  [init_vel,
             aux_vel,
             pick_vel_before,
             pick_vel,
             pick_vel,
             pick_vel_above,
             carry_vel,
             carry_vel,
             carry_vel,
             carry_vel,
             place_vel_above,
             place_vel]  ;
time = [0.0,
        1.0,
        1.5,
        1.9,
        2.5,
        2.9,
        3.5,
        4.7,
        5.9,
        6.2,
        7.3,
        8.2];
    

    picktrajectory = waypointTrajectory(waypoints, ...
    'TimeOfArrival',time, ...
    'Orientation',orientation, ...
    'SampleRate',1000);

[position, orientation, velocity, acceleration, angularVelocity] = picktrajectory()


% waypoints = [0.52, -0.18, 0.59; %init
%              0.7,0.0,0.45;      %aux
%              0.696665, -0.39134 ,0.251281; %before obj
%              0.695176 ,-0.396105 , 0.253126  ; %at obj  ->pregrasp   
%              0.695176 ,-0.396105 , 0.253126  ; %at obj -> after ori change full grasp    
%              0.68,-0.36 , 0.35  ; % above box
%              0.64 , -0.11, 0.403;%carry
%              0.6,0.0,0.43; %carry
%              0.54,0.2,0.4; %carry
%              0.51,0.24,0.407; %carry
%              place_pos_above] %place


% place_pos_above(1) = [0.4758, 0.4375,0.303];
% place_pos(1) = [0.4758, 0.4375,0.22];


% orientation(:,:,1) = [0,-1,0; %init
%                       -1,0,0;
%                        0,0,-1];
% orientation(:,:,2) = [0.52, -0.617,-0.01;
%                       -0.43, 0.121,-0.9895;
%                       0.74, 0.778,0.144];
% orientation(:,:,3) = [0.787, -0.617,-0.01;
%                       0.078, 0.121,-0.9895;
%                       0.6121, 0.778,0.144];   
                  
%                   
%  orientation(:,:,4) = [   0.365173 ,  -0.840191 ,  -0.400909;
%                          -0.00795357  ,  0.427819 ,   -0.90383;
%                               0.930906 ,   0.333243   , 0.149545];
%                           
%  orientation(:,:,5) = [  0.539264  , -0.74728 , -0.388288;
%                         -0.0219643  , 0.448437 , -0.893545;     %cyl ori
%                          0.84185 ,  0.490385 ,  0.225412];
                     
                  
%  orientation(:,:,7) = [  0.66  , -0.67 , 0.335
%                         -0.23  , -0.6 , -0.758;
%                          0.71 ,  0.42,  -0.558]; 
                     
%  orientation(:,:,8) = [  0.383 , -0.888 , 0.251;
%                         -0.62  , -0.449 , -0.643;
%                          -0.685 ,  0.0906996 , -0.723];
%                      
%   orientation(:,:,9) = [  0.9 , -0.4044, 0.131;
%                         -0.42  , -0.879 , 0.218;
%                          0.03,  -0.253 , -0.967];  
%     orientation(:,:,10) = [  0.9 , -0.345, -0.28;
%                         -0.054  , -0.713 , 0.7;
%                          0.44,  -0.61 , -0.66]; 
%      orientation(:,:,11) = [  0.465 , -0.665, -0.584;
%                         0.252  , -0.533 , 0.8;
%                          0.848,  -0.523 , -0.08];
                     
% orientation = [ 0.0, 0.707, -0.707,0;
%                 0.70496, - 0.53911,  0.3664,- 0.27956;
%                 
%                 ]
                
