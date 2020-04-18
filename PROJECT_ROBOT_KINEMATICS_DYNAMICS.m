clc; clear; close all;
load('Data_groove_weld_fv_torus.mat') % This is some data you can use

%% The Robot Task
tube_diameter = 0.3; %m
outer_radius = 0.95; %m
mean_radius = outer_radius-tube_diameter/2; %m
drill_diameter = 0.020; %m

%% Task 1: Drilling 8 Holes
Drill = circle([0,0,0],drill_diameter/2);
Drill = [Drill(:,1:end);ones(size(Drill,2),1)'];
Drill_Transformation = zeros(4,4,8);
Drill2 = zeros(4,size(Drill,2)*8);
i=1;
scale_robot=1;
for theta = 9.5*pi/180:23*pi/180:170.5*pi/180
    Drill_Transformation(:,:,i) = transl(mean_radius*cos(theta),-tube_diameter/2,mean_radius*sin(theta))*trotx(-pi/2)*trotz(pi/2-theta);
    for j=1:size(Drill,2)
        Drill2(:,scale_robot) = Drill_Transformation(:,:,i)*Drill(:,j);
        scale_robot=scale_robot+1;
    end
    i=i+1;
end
figure;
plot3(Drill2(1,:),Drill2(2,:),Drill2(3,:),'r.', 'LineWidth', 2);
hold on; axis equal; box; view(50, 30)
title('Working Points');
xlabel('x'); ylabel('y'); zlabel('z');
for i=1:8
    trplot(Drill_Transformation(:,:,i), 'length', 0.1, 'color', 'b', 'thick', 2);
end

%% Task 2: Drilling and milling 8 shapes
Groove(1:3,:)=Groove(1:3,:)/1000;
Groove_length = max(Groove(2,:))-min(Groove(2,:));
Groove_diameter = max(Groove(1,:))-min(Groove(1,:));
Groove(2,:) = Groove(2,:)+((Groove_length-Groove_diameter)/2);
Groove_Transformation = zeros(4,4,8);
Groove2 = zeros(4,size(Groove,2)*8);
i=1;
scale_robot=1;
for theta = 9.5*pi/180:23*pi/180:170.5*pi/180
    Groove_Transformation (:,:,i) = trotz(pi/2)*transl(0,-outer_radius*cos(theta),outer_radius*sin(theta))*trotx(pi/2-theta);
    for j=1:size(Groove,2)
        Groove2 (:,scale_robot) = Groove_Transformation(:,:,i)*Groove(:,j);
        scale_robot=scale_robot+1;
    end
    i=i+1;
end
plot3(Groove2(1,:),Groove2(2,:),Groove2(3,:),'m.');
for i=1:8
    trplot(Groove_Transformation(:,:,i), 'length', 0.1, 'color', 'r', 'thick', 2);
end

%% Task 3: Welding Points
Weld = [0;0;0;1];
height = pi*mean_radius/8;
alpha_helix = atan(8*height/pi*tube_diameter);
Weld_Transformation = zeros(4,4,64);
Weld2 = zeros(4,size(Weld_Transformation,3));
scale_robot=1;
for i=0:63
    Weld_Transformation(:,:,i+1) = troty(-i*pi/63)*transl(mean_radius,0,0)*transl(tube_diameter/2*cos(i*pi/4),tube_diameter/2*sin(i*pi/4),0)*trotz(-pi/2+i*pi/4)*trotx(pi/2)*trotz(-alpha_helix);
    Weld2 (:,i+1) = Weld_Transformation(:,:,i+1)*Weld;
end
plot3(Weld2(1,:),Weld2(2,:),Weld2(3,:),'g','LineWidth', 2.5);
%scatter3(Weld2(1,:),Weld2(2,:),Weld2(3,:),'b','fillet');
for i=1:64
    trplot(Weld_Transformation(:,:,i), 'length', 0.1, 'color', 'c', 'thick', 2);
end

%% Task 4: Environment Description
a = 0.6; %First dimension of table
b = 2.4; %Second dimension of table

%Coordinates of the table
xA=0.5;
xB=xA+a*cos(30*pi/180);
xC=xB-b*cos(20*pi/180)*sin(30*pi/180);
xD=xC-a*cos(30*pi/180);
yA=xA*tan(30*pi/180);
yB=yA+a*sin(30*pi/180);
yC=yB+b*cos(20*pi/180)*cos(30*pi/180);
yD=yC-a*sin(30*pi/180);
zA=0.75;
zB=zA;
zC=0.75+b*sin(20*pi/180);
zD=zC;

%fv=stlread('Torus.stl')% fv is a struct with faces and vertices
fv.vertices=fv.vertices/10;
ma=max(fv.vertices)
mi=min(fv.vertices)
dmami=ma-mi



%Transformation for the torus to the working points
T1 = transl(-mean_radius,0,0)*trotx(pi);
for i=1:size(fv.vertices)
    torus(:,i) = T1*[fv.vertices(i,:)';1];
    fv.vertices(i,:) = torus(1:3,i)';
end

SS=patch(fv,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');
alpha (SS,0.2)
% Fix the axes scaling, and set a nice view angle
axis('image');

table_center = [(xB+xD)/2; (yB+yD)/2; (zA+zD)/2];
Transf_to_table = transl(table_center(1), table_center(2), table_center(3))*trotz(-60*pi/180)*troty(20*pi/180);
Drill3 = zeros(size(Drill2));
Drill3(:,1:end) = Transf_to_table*Drill2(:,1:end);
Groove3 = zeros(size(Groove2));
Groove3(:,1:end) = Transf_to_table*Groove2(:,1:end);
Weld3 = zeros(size(Weld2));
Weld3(:,1:end) = Transf_to_table*Weld2(:,1:end);
Torus3 = zeros(size(torus));
Torus3(:,1:end) = Transf_to_table*torus(:,1:end);
fv.vertices(1:end,:) = Torus3(1:3,1:end)';

figure;
title('Environment with robot');
xlabel('x'); ylabel('y'); zlabel('z');
hold on; axis equal;
fill3([xA xB xC xD],[yA yB yC yD],[zA zB zC zD],'y');
plot3(Drill3(1,:),Drill3(2,:),Drill3(3,:),'r.'); 
plot3(Groove3(1,:),Groove3(2,:),Groove3(3,:),'m.'); 
plot3(Weld3(1,:),Weld3(2,:),Weld3(3,:),'g','LineWidth',2); 
%scatter3(Weld3(1,:),Weld3(2,:),Weld3(3,:),'b','fillet');

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

%%
%Using the DH parameters for defining the links
scale_robot = 2.2;
L(1) = Link([0, 0, 0, 0],'modified');
L(2) = Link([0, 0, 0, -pi/2],'modified');
L(3) = Link([0, 0.15005*scale_robot, 0.4318*scale_robot, 0],'modified');
L(4) = Link([0, 0.4331*scale_robot, 0.0203*scale_robot, -pi/2],'modified');
L(5) = Link([0, 0, 0, pi/2],'modified');
L(6) = Link([0, 0, 0, -pi/2],'modified');
%Assempling the links

p560m_RKDC = SerialLink(L);

p560m_RKDC = SerialLink(L, 'tool',[1;0;0.5]);
%Viewing its DH parameters

p560m_RKDC                                                       

%Defining a join vector

qz=[0 0 0 0 0 0]

%Checking the Fordward Kinematics

T1=p560m_RKDC.fkine(qz)

%Plotting the Robot with all joint at '0'

p560m_RKDC.plot(qz)
 
%Using a GUI to play with joints

p560m_RKDC.teach