%% The Robot task
% Drilling holes in rear and front faces of an iron piece with shape of half torus. 
% Milling a groove-shape around the outer radius of the torus and finally 
% weld the iron strip joints which conform a spiral with a half-torus shape
% The main dimensions of the torus are: 0.3m of tube diameter and 0.95 m outer radius.
% A specialized machine conform the tube-spiral in a torus shape. 
% See a video of a spiral tubeformer machinery:
% https://www.youtube.com/watch?v=1rvbirDwvRI

close all;
clear all;

load('Data_groove_weld_fv_torus.mat') % This is some data you can use
open('Toro_Robotica.fig') % Rotate the point of view to better understand the task to be done

%% Task 1 Drilling 8 holes 
% Diameter of the drill 0.020 m. Number of holes: 8 in the front face.
%% Task 2. Drilling and milling 8 shapes
% Numers of groove 8 in the outer diameter
% The groove has the following shape in milimeteres. 
figure
plot3(Groove(1,:),Groove(2,:),Groove(3,:),'r') % plotting the Groove
title ('Groove shape')
xlabel('x');
ylabel('y');
zlabel('z');
axis equal
%% Task 3 Welding points
% The Torus part is conformed from a iron strip that generate a spiral-like tube. 
% This assembling must be weld to fixe the structure and gain in rigidity.
% The manipulation of the machine outputs a Torus of 8 turns and dimensions: 0.3m of tube diameter and 0.95 m outer radius
% It is planned to first weld every 117.8 mm.
% See the solution for better understanding.
figure
plot3(Weld_points(1,:),Weld_points(2,:),Weld_points(3,:),'g','LineWidth',2)
hold on
scatter3(Weld_points(1,:),Weld_points(2,:),Weld_points(3,:),'b','filled')
xlabel('x');
ylabel('y');
zlabel('z');
axis 'equal'
%% Enviroment drescription
% The torus 'Pose' must be facing up and placed in the midle of a tilted table (20º) of
% 0.6m x 1.2 m that is rotated 30º wrt the x axis of the robot frame. 
% The height of lower dimension is 0.75 m. 
% Use the Matlab command fill3 to visualize 4 faces of the tabel and sketch
% easily the enviroment
figure
xlabel('x');
ylabel('y');
zlabel('z');
axis 'equal'
fill3([1 3 3 1],[0 0 5 5],[4 4 6 6],'r');
alpha 0.3
% 
%% Reading '.stl' file
fv=stlread('Toro_Robotica.stl');% fv is a struct with faces and vertices
fv.vertices=fv.vertices;
ma=max(fv.vertices);
mi=min(fv.vertices);
dmami=ma-mi;
%% Visualizing STL figures 
figure
SS=patch(fv,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');
alpha (SS,0.2)
view(30,30)
% Fix the axes scaling, and set a nice view angle
axis('image');
axis 'equal'
%% Help 1: Think about managing RTB data extructe.
% Drawing 30 points of circle at center point [2 3 4]', radius 5, using RTB data extructure
n=30;px=2;py=3;pz=4;r=5;
for i=1:n
Ptos(:,:,i)=transl(px,py,pz)*trotz(2*pi*i/n)*transl(5,0,0)
end
coor_circle=transl(Ptos)'
figure
scatter3(coor_circle(1,:),coor_circle(2,:),coor_circle(3,:),'r','LineWidth',2)
%% Help 2: Think about operator
ptos=circle([0 0 0], 1.5 ,16)
ptosHT=[ptos;ones(1,50)]
figure
scatter3(ptosHT(1,:),ptosHT(2,:),ptosHT(3,:),'filled')
hold on
ptos_trans=transl(0,0,-6)*troty(pi/2)*ptosHT
scatter3(ptos_trans(1,:),ptos_trans(2,:),ptos_trans(3,:),'filled')
ptos_p4=troty(pi/4)*ptos_trans
scatter3(ptos_p4(1,:),ptos_p4(2,:),ptos_p4(3,:),'filled')
ptos_p2=troty(pi/2)*ptos_trans
scatter3(ptos_p2(1,:),ptos_p2(2,:),ptos_p2(3,:),'filled')
view (5,20)
%% Help 3. What about a spiral in a cilinder
n=30;px=2;py=3;pz=4;r=5;h=10
figure
for i=1:n
Ptos(:,:,i)=transl(px,py,pz)*trotz(2*pi*i/n)*transl(5,0,0)*transl(0,0,h*i/n)
end
coor_circle=transl(Ptos)'
figure
scatter3(coor_circle(1,:),coor_circle(2,:),coor_circle(3,:),'r','LineWidth',2)
view (25,50)
hold off
%% This code inspire you for torus Helix??
% I hope yes!!

%% Building your on robot: Start with a Puma 560.
% Go to example at ATENEA 'Building Puma560 using RTB'
% Visit the pdf 'robot.pdf' at the RTB folder.
% You can also modify an exixting puma robot.
%% Using Invers Kinematic or manualy drive the robot to the target pose
% Even though we still did not explain the inverse Kinematics theory 
% you can use the function 'robot.ikine' to get joint configuration based on 
% desired robot poses.
% See the example: Example of a Puma560 doing a Task.
% Be aware of the possible singularities and think about the shape of the
% tool.
% Or you can use the teach GUI to get the joint configuration. 
%% Computing torques for static force at the tip of the Puma robot
% Task 2 & 3 -Drilling & Milling.
% Previous experiment shows that for drilling the tool placed at the EE will needed 
% 20N normal to surface  to perform a hole. For the milling will be necesary 40N 
% tangential to the surface of minimum curvature as shown in the above
% figure.
% I strongly recomend you that before solving this question review again the slices of theory and
% complete the example for the 3R Manipulator and visit the code example about Jacobians.
