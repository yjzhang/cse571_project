scale = 0.1;
x_cent = 1.5;
y_cent = 1.5;
pose  = pi/2;
rad = 0.2;
noise = 0.01;
M=createmap('map1.jpg');
scrsz = get(groot,'ScreenSize');
figure('Position',[scrsz(1) scrsz(2) scrsz(3)/2 scrsz(4)/2])
subplot(1,3,1)
printmap(M,scale);
Msense=ones(100,100);
for i=1:1:70
subplot(1,3,1)
printmap(M,scale);
print_robot(x_cent,y_cent,pose,rad);
subplot(1,3,2)
[Msense Mcur]= raytrace(x_cent,y_cent,pose*180/pi,M,5,Msense,scale,noise);
printmap(Msense,scale);
print_robot(x_cent,y_cent,pose,rad);
subplot(1,3,3)
printmap(Mcur,scale);
print_robot(x_cent,y_cent,pose,rad);
if(i>35)
    x_cent=x_cent+0.2;
    pose=0;
else
    y_cent=y_cent+0.2;
end
a=strcat('Animation',num2str(i));
print(a,'-djpeg')
end


