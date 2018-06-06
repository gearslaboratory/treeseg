%Andrew Burt - a.burt.12@ucl.ac.uk

function subCloudCyl(cloud,model)
    h = figure('visible','off');
    %campos([-20,18,8]);
    view([0 30])
    hold on;
    axis equal;
    grid off;
    axis off;
    for i=1:length(infile)
        data = load(infile{i});
        scatter3(data(:,1),data(:,2),data(:,3),0.1,rand(1,3));
    end
    
    
    
    
    
    
    ti = get(gca,'TightInset');
    set(gca,'Position',[ti(1) ti(2) 1-ti(3)-ti(1) 1-ti(4)-ti(2)]);
    set(gca,'units','centimeters')
    pos = get(gca,'Position');
    ti = get(gca,'TightInset');
    set(gcf, 'PaperUnits','centimeters');
    set(gcf, 'PaperSize', [pos(3)+ti(1)+ti(3) pos(4)+ti(2)+ti(4)]);
    set(gcf, 'PaperPositionMode', 'manual');
    set(gcf, 'PaperPosition',[0 0 pos(3)+ti(1)+ti(3) pos(4)+ti(2)+ti(4)]);
    print(h,'-dpdf','-r600','clouds.pdf');
end
