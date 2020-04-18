n=30;px=2;py=3;pz=4;r=5;
for i=1:n
Ptos(:,:,i)=transl(px,py,pz)*trotz(2*pi*i/n)*transl(5,0,0)
end
coor_circle=transl(Ptos)'
figure
scatter3(coor_circle(1,:),coor_circle(2,:),coor_circle(3,:),'r','LineWidth',2)
