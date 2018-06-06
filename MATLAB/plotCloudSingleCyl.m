%Andrew Burt - a.burt.12@ucl.ac.uk

function plotCloudSingleCyl(cloud,model,cylinder )

data = load(cloud);
load(model)
plotCloud(cloud);
hold on;
cylinder2P(Rad(cylinder),20,[Sta(cylinder,1),Sta(cylinder,2),Sta(cylinder,3)],[Sta(cylinder,1)+Len(cylinder)*Axe(cylinder,1),Sta(cylinder,2)+Len(cylinder)*Axe(cylinder,2),Sta(cylinder,3)+Len(cylinder)*Axe(cylinder,3)]);
alpha(0.1);
view([140,60])
print('cyl_0.1', '-dpng', '-r300');
end

