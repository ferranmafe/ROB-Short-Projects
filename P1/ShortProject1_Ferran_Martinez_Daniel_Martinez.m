%% SHORT PROJECT KINEMATICS - ROB

% Authors: Ferran Martinez Felipe
%          Daniel Martinez Bordes

%% 1 - Sketching the enviroment of the robotics work cell.
% It is spected: main reference frames. 
% Plot the robot Puma, draw the working table and the torus in working position.
% Give diferent points of view of the scenary: Top, Front, Lateral and isometrics view.
close all;
clear all;

%% 1.1 - Load torus and calculate bounding box parameters
torus = stlread('Toro_Robotica.stl');
% We calculate the bounding box of the given torus in order to move it to
% the origin
boundingBoxMax = max(torus.vertices);
boundingBoxMin = min(torus.vertices);
boundingBoxDiag = boundingBoxMax - boundingBoxMin;
boundingBoxBase = (boundingBoxMax + boundingBoxMin) / 2;
% We take as the basis the max of the boundingBox because the torus is
% given inversed
boundingBoxBase(2) = boundingBoxMax(2);

%% 1.2 - Define table
tableMin = [1 0 4];
tableMax = [3 5 6];
tableCenter = (tableMin + tableMax)/2;
tableP1 = [tableMin(1) tableMax(1) tableMax(1) tableMin(1)];
tableP2 = [tableMin(2) tableMin(2) tableMax(2) tableMax(2)]; 
tableP3 = [tableMin(3) tableMin(3) tableMax(3) tableMax(3)];

%% 1.3 -  Moving Torus to the Origin(0,0,0)
% We need to translate our torus toto the origin of our cordinates system
[n, m] = size(torus.vertices);
fv = [torus.vertices ones(n,1)]';
% Translate to the origin
fvOri = transl(-boundingBoxBase) * fv;

%% 1.4 - Move Torus to the top of the table.
% Angles needed to have the Tours in the origin oriented as we want to
angleX = -pi/2; % Stand it up
angleY = 0;
angleZ = pi/2; % Put it parallel to the slope of the table
% Once oriented we perform an extra rotation to fit the table angle
% We computate the angle of the table (axis-X is were the inclination goes)
tableAngle = atan2((tableMax(3) - tableMin(3)), (tableMax(2) - tableMin(2)));

% Move the torus to the table, we apply the rotations in the origin +
% translation to the center of the table (its base)
% We save this transformations beacuse we will apply it to all the elements
% later
transfToTable = transl(tableCenter) * trotx(tableAngle);
fvTransf = transfToTable * trotz(angleZ) * troty(angleY) * trotx(angleX) * fvOri;
fvFinal = fvTransf';
% We assign back the transformed vertices to the torus structure
torus.vertices = fvFinal(:, 1:3);

%% 1.5 - Display torus and table
% We will print one isometric view and 3 additional views in a subplot (top/front/lateral)
%% 1.5.1 - Isometric view
% Table
figure(1);
fill3(tableP1,tableP2,tableP3,'r');
alpha 0.3 %To mke it transparent
hold on;
%Torus
SS=patch(torus,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');
alpha (SS,0.2)
% Fix the axes scaling, and set a nice view angle
view(30,30);
axis('image');
xlabel('x');
ylabel('y');
zlabel('z');
title('Isometric view');

%% 1.5.2 -  Frontal view
% Table
figure (2)
subplot(2,2,1);
fill3(tableP1,tableP2,tableP3,'r');
alpha 0.3
hold on;
%Torus
SS=patch(torus,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');
alpha (SS,0.2)
% Fix the axes scaling, and set a nice view angle
view(0,0);
axis('image');
xlabel('x');
ylabel('y');
zlabel('z');
title('Frontal view');

%% 1.5.3 -  Lateral view
% Table
subplot(2,2,[3 4]);
fill3(tableP1,tableP2,tableP3,'r');
alpha 0.3
hold on;
%Torus
SS=patch(torus,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');
alpha (SS,0.2)
% Fix the axes scaling, and set a nice view angle
view(90,0);
axis('image');
xlabel('x');
ylabel('y');
zlabel('z');
title('Lateral view');

%% 1.5.4 -  Top view
% Table
subplot(2,2,2);
fill3(tableP1,tableP2,tableP3,'r');
alpha 0.3
hold on;
%Torus
SS=patch(torus,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');
alpha (SS,0.2)
% Fix the axes scaling, and set a nice view angle
view(0,90);
axis('image');
xlabel('x');
ylabel('y');
zlabel('z');
title('Top view');

%% 2 - Working points.
% Give here your code to get the variables to locate:

%Torus parameters:
torusCilDiam = 0.2;
torusDiam = 1.6;
helixLength = 0.8;
torusCilRadius = torusCilDiam/2;
torusRadius = torusDiam/2;
helixAngle = atan2(helixLength, torusDiam);

%Groove parameters:
load('Data_groove_weld_fv_torus.mat');
grooveBoundingBoxMax = max(Groove');
grooveBoundingBoxMin = min(Groove');
grooveBoundingBoxCenter = (grooveBoundingBoxMax + grooveBoundingBoxMin) / 2;
grooveDiam = 0.02;
scaleValue = grooveDiam / (grooveBoundingBoxMax(1)-grooveBoundingBoxMin(1));
grooveNum = 8;

%Circle parameters:
circleNum = 8;
circleDiam = 0.02;
circleRadius = circleDiam/2;
circleAngles = linspace(0, 2*pi, 65);
circleAngles = circleAngles(1:64);
Circle = [circleRadius * cos(circleAngles); circleRadius * sin(circleAngles); zeros(1,64); ones(1,64)];

%Welding-points parameters:
weldPointsNum = 64;
weldPointsHelixNum = 8;
P = [0 0 0 1]';
helixSection = 2 * sin(pi / (2 * weldPointsNum)) * torusRadius;
cilSection = 2 * sin((2 * pi) / (2 * weldPointsHelixNum)) * torusCilRadius;
helixTrajAngle = atan2(helixSection, cilSection);

%% 2.1 - The reference frame for all drills holes, such that z-axis is orthogonal
% to the surface of the torus and the x-axis is in the direction of minimun
% curvature. Draw in scale the frames
figure(3)
% Table
fill3(tableP1,tableP2,tableP3,'r');
alpha 0.3
hold on;
%Torus
SS=patch(torus,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');
alpha (SS,0.3)
% Fix the axes scaling, and set a nice view angle
view(30,30);
axis('image');
xlabel('x');
ylabel('y');
zlabel('z');
title('Lateral circle frames');
hold on;

% Circles face 1:
% We calculate the rotation we need to do to the piece between 0 an pi
% radians to the semicircle of the torus, as it is an helix we also need to
% do a translate in every increment.
rotCircleAngle = linspace(0, pi, circleNum);
transCircleDist = linspace(-helixLength/2, helixLength/2, circleNum);
circleFrames1 = zeros(4,4,circleNum);
Circles1 = [];
for i = 1: circleNum
    % Store the frame, orthogonal to the surface the z-axis!
    % Rotate z axis to be orth, rotate-y to ve vertical, translate to the
    % origin of the torus anf from that position at every iteration we
    % rotate a angle inclrement and translation.
    circleFrames1(:,:,i) = transfToTable * transl([transCircleDist(i) 0 0]) * trotx(-rotCircleAngle(i)) * transl([torusCilRadius -torusRadius 0]) * troty(pi/2 + helixAngle) * trotz(pi);
    % Store the circle points in the position we need to plot them
    Circles1(:,:,i) =  circleFrames1(:,:,i) * Circle;
    hold on;
    %Ploting the circle ant its frame
    plot3(Circles1(1,:,i),Circles1(2,:,i),Circles1(3,:,i),'r','linewidth', 1)
    trplot(circleFrames1(:,:,i),'length', 0.3);
end

%Circles face 2: (analogous but with a translation to the other side of the torus cylinder)
rotCircleAngle = linspace(0, pi, circleNum);
transCircleDist = linspace(-helixLength/2, helixLength/2, circleNum);
circleFrames2 = zeros(4,4,circleNum);
Circles2 = [];
for i = 1:circleNum
    circleFrames2(:,:,i) = transfToTable * transl([transCircleDist(i) 0 0]) * trotx(-rotCircleAngle(i)) * transl([-torusCilRadius -torusRadius 0]) * troty(-pi/2 + helixAngle) * trotz(pi);
    Circles2(:,:,i) = circleFrames2(:,:,i) * Circle;
    hold on;
    plot3(Circles2(1,:,i),Circles2(2,:,i),Circles2(3,:,i),'r','linewidth', 1)
    trplot(circleFrames2(:,:,i),'length', 0.3);
end


%% 2.2 - The reference frame for all milling grooves, such that z-axis is orthogonal
figure(4)
% Table
fill3(tableP1,tableP2,tableP3,'r');
alpha 0.3
hold on;
%Torus
SS=patch(torus,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');
alpha (SS,0.3)
% Fix the axes scaling, and set a nice view angle
view(30,30);
axis('image');
xlabel('x');
ylabel('y');
zlabel('z');
title('Groove frames');
hold on;

% Milling Grooves
% Same steps than before but now we need to scale the groove also and the
% rotation goes form -pi/2 to pi/2 case we move the piece first to the top
% of the torus and then we apply the rotation+ translation, there is an
% extra rotation because the torus has an helixAngle.
rotGrooveAngle = linspace(pi/2, -pi/2, grooveNum);
transGrooveDist = linspace(-helixLength/2, helixLength/2, grooveNum);
grooveFrames = zeros(4,4,grooveNum);
Grooves = [];
for i = 1:grooveNum
    % We save the frame
    grooveFrames(:,:,i) = transfToTable * transl([transGrooveDist(i) 0 0]) * trotx(rotGrooveAngle(i)) * transl([0 0 (torusRadius + torusCilRadius)]) * trotz(-helixAngle);
    % We also save the point to plot it later on
    Grooves(:,:,i) = grooveFrames(:,:,i) * trscale(scaleValue) * transl(-grooveBoundingBoxCenter(1:3)) * Groove;
    hold on;
    % Plot groove and frame
    plot3(Grooves(1,:,i),Grooves(2,:,i),Grooves(3,:,i),'r','linewidth', 1)
    trplot(grooveFrames(:,:,i),'length', 0.3);
end


%% 2.3 - The reference frames for all welding points
% such that z-axis of the tool is orthogonal to the surface of the torus
% and the x-axis is in the direction of spiral trajectory. Draw in scale the frames

figure(5)
% Table
fill3(tableP1,tableP2,tableP3,'r');
alpha 0.3
hold on;
%Torus
SS=patch(torus,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');
alpha (SS,0.3)
% Fix the axes scaling, and set a nice view angle
view(30,30);
axis('image');
xlabel('x');
ylabel('y');
zlabel('z');
title('Welding points frames');
hold on;

% Welding points
% In this case we move a virtual point in the origin to certain positions,
% doing an helix outside the torus figure. To do that we need an additional
% rotation but in this case respect to the cylinder circumference
rotTorusAngle = linspace(0, pi, weldPointsNum);
transTorusDist = linspace(-helixLength/2, helixLength/2, weldPointsNum);
circumferenceAngles = linspace(0, 2*pi, weldPointsHelixNum + 1);
weldFrames = zeros(4,4,weldPointsNum);
for i = 1:weldPointsNum
    j = mod(i-1, weldPointsHelixNum) + 1;
    weldFrames(:,:,i) = transfToTable * trotx(-rotTorusAngle(i)) * transl([transTorusDist(i) -torusRadius 0]) * trotz(circumferenceAngles(j)) * transl([0 -torusCilRadius 0]) * trotx(pi/2) * trotz(helixTrajAngle);
    weldPoints(:,i) = weldFrames(:,:,i) * P;
    trplot(weldFrames(:,:,i),'length', 0.3);
    hold on
end
hold on
plot3(weldPoints(1,:),weldPoints(2,:),weldPoints(3,:),'g','LineWidth',1.5)
hold on
scatter3(weldPoints(1,:),weldPoints(2,:),weldPoints(3,:),'b','filled')

%% 2.4 - Torus with circular points, welding points amd grooves
% We finally plot the figure in a good position without the frames for
% clarity
figure(6)
% Table
fill3(tableP1,tableP2,tableP3,'r');
alpha 0.3
hold on;
%Torus
SS=patch(torus,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');
alpha (SS,0.3)
% Fix the axes scaling, and set a nice view angle
view(30,30);
axis('image');
xlabel('x');
ylabel('y');
zlabel('z');
title('Complete torus');
hold on

% We plot everything using the saved data
%Welding points draw
plot3(weldPoints(1,:),weldPoints(2,:),weldPoints(3,:),'g','LineWidth',1.5)
hold on
scatter3(weldPoints(1,:),weldPoints(2,:),weldPoints(3,:),'b','filled')
hold on
%Grooves draw
for i = 1:grooveNum
    plot3(Grooves(1,:,i),Grooves(2,:,i),Grooves(3,:,i),'r','linewidth', 1)
    hold on;
end
%Circles1 draw
for i = 1: circleNum
    plot3(Circles1(1,:,i),Circles1(2,:,i),Circles1(3,:,i),'r','linewidth', 1)
    hold on;
end

%Circles2 draw:
for i = 1:circleNum
    plot3(Circles2(1,:,i),Circles2(2,:,i),Circles2(3,:,i),'r','linewidth', 1)
    hold on;
end

%% Computing motor torques for the static forces.
% Give here your code to fill two tables with the motor torque at each robot pose:
% Table 1 (6x16): Rows are the motor torques (6x1). Columns (1x16) are the labeled drills
% including the initial drill before milling.
% Table 2 (6x8): Rows are the motor torques (6x1). Columns (1x8) are the
% labeled milling of the groove.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% We print all the scene again, the tables needed by the statement of the
% problem are the result of applying the inverse kinematics to all the
% frames we have. We will store also security positions a little bit
% distant to the interest point to avoid potential crashes.
figure(7)
% Table
fill3(tableP1,tableP2,tableP3,'r');
alpha 0.3
hold on;
%Torus
SS=patch(torus,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');
alpha (SS,0.3)
% Fix the axes scaling, and set a nice view angle
view(30,30);
%axis('equal');
ylim ([1 4]);
xlim ([0 4]);
zlim ([4.5 7]);
xlabel('x');
ylabel('y');
zlabel('z');
title('Robot doing task');
hold on
%Welding points draw
plot3(weldPoints(1,:),weldPoints(2,:),weldPoints(3,:),'g','LineWidth',1.5)
hold on
scatter3(weldPoints(1,:),weldPoints(2,:),weldPoints(3,:),'b','filled')
hold on
%Grooves draw
for i = 1:grooveNum
    plot3(Grooves(1,:,i),Grooves(2,:,i),Grooves(3,:,i),'r','linewidth', 1)
    hold on;
end
%Circles1 draw
for i = 1: circleNum
    plot3(Circles1(1,:,i),Circles1(2,:,i),Circles1(3,:,i),'r','linewidth', 1)
    hold on;
end

%Circles2 draw:
for i = 1:circleNum
    plot3(Circles2(1,:,i),Circles2(2,:,i),Circles2(3,:,i),'r','linewidth', 1)
    hold on;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Robot instance
mdl_puma560
qz=[0 0 0 0 0 0];
% Move the base to pot it in to the table to be capable to work, and plot
% initial position
robotPose1 = transfToTable * transl([1.5 -0.7 0.5]) * troty(pi);
robotPose2 = transfToTable * transl([-1.5 0.6 0.5]) * troty(pi);

p560.base = robotPose1;

p560.links(1,4).d = 1.5;
p560.tool = transl(0, 0, 0.3);
p560.plot(qz)


% We calculate the inverse kinematics for every point, is the table with 
% the values for every link and join, we also calculate the control
% positions by multiplying with a translation matrix by the right part of
% the frame.
grooveIncY = ((scaleValue * (grooveBoundingBoxMax(2) - grooveBoundingBoxMin(2))) - grooveDiam) /2; % Li traiem al component y la part corresponent a el radi del cercle i dividim en 2 parts
controlIncZ = 0.1;

% InvKin circles:
%Face 1:

circleInvKin1 = zeros(circleNum, 6);
circleInvKinControl1 = zeros(circleNum, 6);
for i = 1:circleNum
    % We colocate the robot into the position 1
    p560.base = robotPose1;
    
    % Next, we calculate the inverse kinematics for the circle and the
    % secure position related to it.
    circleInvKin1(i,:) = p560.ikine6s(circleFrames1(:,:,i) * trotx(pi) , 'run');
    circleInvKinControl1(i,:) = p560.ikine6s(circleFrames1(:,:,i) * transl(0,0,controlIncZ) * trotx(pi) , 'run');
    
    % If we don't obtain a NaN in some of the ikine functions, the point is
    % reachable. Then we must plot the arm going to the point (with sleeps to
    % perceive better the approximation)
    if (~(prod(isnan(circleInvKin1(i,:))) || prod(isnan(circleInvKinControl1(i,:)))))
        p560.plot(circleInvKinControl1(i,:));
        pause(0.1);
        p560.plot(circleInvKin1(i,:));
        pause(0.1);
        p560.plot(circleInvKinControl1(i,:));
        pause(0.1);
    end
end   

% InvKin Welding-points:
% In this case we calculate the inverse kinematics for all the welding
% points, assuming that at least half of them will be NaN due the position
% of the robot (but it's easier than split the welding points)
weldInvKin = p560.ikine6s(weldFrames, 'run');
weldInvKinControl = zeros(weldPointsNum, 6);
for i = 1:weldPointsNum
    % We make sure that the robot is in robotPose1 (but it's not really
    % needed because the robot is in robotPose1)
    p560.base = robotPose1;
    
    if mod(i-1, weldPointsHelixNum) < weldPointsHelixNum/2
        % We calculate the inverse kinematics for the welding points
        weldInvKin(i,:) = p560.ikine6s(weldFrames(:,:,i) * trotx(pi), 'run');
        weldInvKinControl(i,:) = p560.ikine6s(weldFrames(:,:,i) * transl(0,0,controlIncZ) * trotx(pi), 'run');

        % Finally we check if the point is NaN. If not, we plot the movement.
        if (~(prod(isnan(weldInvKin(i,:))) || prod(isnan(weldInvKinControl(i,:)))))
            p560.plot(weldInvKinControl(i,:));
            pause(0.1);
            p560.plot(weldInvKin(i,:));
            pause(0.1);
            p560.plot(weldInvKinControl(i,:));
            pause(0.1);
        end
    end
end 

%Face 2:
% Plot of the circles of the other face
circleInvKin2 = zeros(circleNum, 6);
circleInvKinControl2 = zeros(circleNum, 6);
for i = 1:circleNum
    % We change the robot position to robotPose2
    p560.base = robotPose2;
    
    % We calculate the inverse kinematics for the each frame
    circleInvKin2(i,:) = p560.ikine6s(circleFrames2(:,:,i) * trotx(pi), 'run');
    circleInvKinControl2(i,:) = p560.ikine6s(circleFrames2(:,:,i) * transl(0,0,controlIncZ) * trotx(pi), 'run');
    
    % If the ikine result is not NaN, we plot the robot movement
    if (~(prod(isnan(circleInvKin2(i,:))) || prod(isnan(circleInvKinControl2(i,:)))))
        p560.plot(circleInvKinControl2(i,:));
        pause(0.1);
        p560.plot(circleInvKin2(i,:));
        pause(0.1);
        p560.plot(circleInvKinControl2(i,:));
        pause(0.1);
    end
end 

% InvKin Grooves: The control position now has an increment of Y because
% the tool has to drill and groove so we need to simulate it, that is why
% we have 4 points (2 to do the work, and the initial and final control positions)
grooveInvKin = zeros(4 * grooveNum, 6);
for i = 0:(grooveNum-1)
    p560.base = robotPose2; 
    % In this case we calculate 4 positions to move (secure position and left,
    % right and center of the groove)
    grooveInvKin(4 * i + 1,:) = p560.ikine6s(grooveFrames(:,:,i+1) * transl(0,-grooveIncY, controlIncZ) * trotx(pi), 'run');
    grooveInvKin(4 * i + 2,:) = p560.ikine6s(grooveFrames(:,:,i+1) * transl(0,-grooveIncY, 0) * trotx(pi), 'run');
    grooveInvKin(4 * i + 3,:) = p560.ikine6s(grooveFrames(:,:,i+1) * transl(0, grooveIncY, 0) * trotx(pi), 'run');
    grooveInvKin(4 * i + 4,:) = p560.ikine6s(grooveFrames(:,:,i+1) * transl(0, grooveIncY, controlIncZ) * trotx(pi), 'run');
    % If none of the ikine results is NaN, we plot the movement 
    if (~(prod(isnan(grooveInvKin(4 * i + 1,:)))) && ~(prod(isnan(grooveInvKin(4 * i + 2,:)))) && ~(prod(isnan(grooveInvKin(4 * i + 3,:)))) && ~(prod(isnan(grooveInvKin(4 * i + 4,:)))))
        p560.plot(grooveInvKin(4 * i + 1,:));
        pause(0.1);
        p560.plot(grooveInvKin(4 * i + 2,:));
        pause(0.1);
        p560.plot(grooveInvKin(4 * i + 3,:));
        pause(0.1);
        p560.plot(grooveInvKin(4 * i + 4,:));
        pause(0.1);
       
    end
    
end 


weldInvKin = p560.ikine6s(weldFrames, 'run');
weldInvKinControl = zeros(weldPointsNum, 6);
% Finally we recalculate the inverse kinematics for the rest of the welding
% points. We use the same procedure than in the other side (calculate all the 
% movements for all the welding points and moving only towards the side correct
% size, because the others are unreachable)
for i = 1:weldPointsNum
    p560.base = robotPose2;
    
    if mod(i-1, weldPointsHelixNum) >= weldPointsHelixNum/2
        % We calculate the inverse kinematics for each of the points
        weldInvKin(i,:) = p560.ikine6s(weldFrames(:,:,i) * trotx(pi), 'run');
        weldInvKinControl(i,:) = p560.ikine6s(weldFrames(:,:,i) * transl(0,0,controlIncZ) * trotx(pi), 'run');

        % If the movement is not NaN, we make the movement
        if (~(prod(isnan(weldInvKin(i,:))) || prod(isnan(weldInvKinControl(i,:)))))
            p560.plot(weldInvKinControl(i,:));
            pause(0.1);
            p560.plot(weldInvKin(i,:));
            pause(0.1);
            p560.plot(weldInvKinControl(i,:));
            pause(0.1);
        end
    end
end 
% Ya esta todo el inverse kin process calculado, simplemente hay que pone 2
% robots ahora y pauses entre los plots para que se vea bien.


