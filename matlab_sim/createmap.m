function M = createmap(filename)
img = imread(filename);
Mtemp=rgb2gray(img);
for i=1:size(Mtemp,1)
    for j=1:size(Mtemp,2)
        if(Mtemp(i,j)>10)
            M(i,j)=1;
        else
            M(i,j)=0;
        end
    end
end
clear Mtemp
end

