
function plotposition(mobileLoc)
    
    %disp("Plot: " + mobileLoc);
        
    networkSize = 2; %adjust for size of room/anchor position
    anchorLoc   = [0 0; networkSize 0; 0 networkSize; networkSize networkSize];
    plot(anchorLoc(:,1),anchorLoc(:,2),'ko','MarkerSize',8,'lineWidth',2,'MarkerFaceColor','k');
    hold on; grid on;
        
    plot(mobileLoc(1),mobileLoc(2),'b+','MarkerSize',8,'lineWidth',2);
    hold off; drawnow;
end
