%Andrew Burt - a.burt.12@ucl.ac.uk

function animateTree(infile1,infile2,infile3,infile4,type,elevation,azimuth_range)

    if(strcmp(type,'cloud')==1)
        data1 = load(infile1);
        data2 = load(infile2);
        data3 = load(infile3);
        data4 = load(infile4);
        counter = 1;
        for azimuth=azimuth_range(1):azimuth_range(2)
            disp(azimuth);
            h = figure(1);
            hold on;
            set(h, 'Position', [1 1 900 650]);
            %whitebg();
            colormap(jet);
            scatter3(data1(:,1),data1(:,2),data1(:,3),1,data1(:,3));
            scatter3(data2(:,1),data2(:,2),data2(:,3),1,data2(:,3));
            scatter3(data3(:,1),data3(:,2),data3(:,3),1,data3(:,3));
            scatter3(data4(:,1),data4(:,2),data4(:,3),1,data4(:,3));
            view([azimuth,elevation]);
            axis equal;
            grid on;
            hold off;
            M(counter) = getframe(gcf);
            clf;
            counter = counter + 1;
        end
        movie2avi(M,'cloud.avi');
        clear all;
    end

    if(strcmp(type,'cylinder')==1)
        counter = 1;
        for azimuth=azimuth_range(1):azimuth_range(2)
            disp(azimuth);
            h = figure(1);
            hold on;
            set(h, 'Position', [1 1 900 650]);
            colormap(jet);
            plotCyls(infile1,1000);
            plotCyls(infile2,1000);
            plotCyls(infile3,1000);
            plotCyls(infile4,1000);
            view([azimuth,elevation]);
            axis equal;
            grid on;
            %whitebg();
            hold off;
            M(counter) = getframe(gcf);
            clf;
            counter = counter + 1;
        end
        movie2avi(M,'cylinder.avi');
    end
    
end
