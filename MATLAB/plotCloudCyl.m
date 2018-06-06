%Andrew Burt - a.burt.12@ucl.ac.uk

function plotCloudCyl(cloud,cylinder)
    plotCloud(cloud);
    hold on;
    plotCyls(cylinder,100);
    alpha(0.1);
end
