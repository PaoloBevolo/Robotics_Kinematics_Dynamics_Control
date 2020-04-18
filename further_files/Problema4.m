clc; clear all; close all;

%datos
l=246.6;
L=399.8;

t= 0:pi/16:2*pi;
r=15.6;
pt=[r*cos(t);r*sin(t);abs(r*cos(t));ones(1,length(t))];

%scatter3(pt(1,:),pt(2,:),pt(3,:),'b','LineWidth',2);
%trplot(pt(:,:), 'length',0.1, 'color','r','thick',2);

T1=zeros(4,4,8);
T2=zeros(4,4,8);
co=1;
for i=-1:2:1
Tl(:,:,co)= transl(0,0,i*((L/2)-r));

    for teta= 0:pi/2:2*pi
    T2(:,:,co)= T1*trotx((i+1)*pi)*trotz(teta)*trotx(-i*pi/4)*transl((l-r)*cos(teta),((l/2)-r)*sin(teta),0);
    pt2(:,:,co)= T2*pt;
    end
    
    co=co+1;
end

scatter3(pt2(1,:),pt2(2,:),pt2(3,:),'b','LineWidth',2);