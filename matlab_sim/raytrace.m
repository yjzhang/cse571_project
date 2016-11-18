function [Mupd Mnew] = raytrace(x_cent, y_cent, pose, M, dtheta, Mprev,scale,noise)
raynum = 360/dtheta;
r=0:0.1:1.2;
Mupd=Mprev;
Mnew = ones(size(M,1),size(M,2));
for i = 1:raynum
    x(i,:)=x_cent+round(r*cosd(pose+(i-1)*dtheta),1);
    y(i,:)=y_cent+round(r*sind(pose+(i-1)*dtheta),1);
end
for i = 1:raynum
    for j = 1:size(r,2)
        inx = round(x(i,j)/scale);
        iny = round(y(i,j)/scale);
        if(M(inx,iny)==0)
            inkx=round(inx+(round(normrnd(0,noise),1)/scale));
            inky=round(iny+(round(normrnd(0,noise),1)/scale));
            if(rand<0.005)
                inkx=round(x(i,end)/scale);
                inky=round(y(i,end)/scale);
            end
            Mnew(inkx,inky)= 0;
            Mupd(inkx,inky)= 0;
            Mnew(inkx+1,inky)=min([0.7  Mnew(inkx+1,inky)]);
            Mupd(inkx+1,inky)=min([0.7  Mupd(inkx+1,inky)]);
            Mnew(inkx-1,inky)=min([0.7  Mnew(inkx-1,inky)]);
            Mupd(inkx-1,inky)=min([0.7  Mupd(inkx-1,inky)]);
            Mnew(inkx,inky+1)=min([0.7  Mnew(inkx,inky+1)]);
            Mupd(inkx,inky+1)=min([0.7  Mupd(inkx,inky+1)]);
            Mnew(inkx,inky-1)=min([0.7  Mnew(inkx,inky-1)]);
            Mupd(inkx,inky-1)=min([0.7  Mupd(inkx,inky-1)]);
            break;
        end
    end
end
           
        
