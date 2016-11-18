function printmap(M,scale)
for i = 1:size(M,1)
    for j = 1:size(M,2)
        rectangle('Position',[scale*i scale*j scale scale],'FaceColor',M(i,j)*ones(1,3),'LineStyle','none')
        hold on
    end
end
axis([0 scale*size(M,1) 0 scale*size(M,2)]);
end
      
        