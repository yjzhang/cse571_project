function print_robot(x_cent,y_cent,pose,rad)
theta = 0:0.01:2*pi;
x=0:0.01:rad;
xcir = x_cent+rad*sin(theta);
ycir = y_cent+rad*cos(theta);
plot(xcir,ycir)
hold on
quiver(x_cent,y_cent,rad*cos(pose),rad*sin(pose))
set(gca,'xtick',[],'ytick',[])
end
