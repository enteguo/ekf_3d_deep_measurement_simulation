oxts = loadOxtsliteData('E:\dataset\2011_10_03_drive_0027_sync');
pose = convertOxtsToPose(oxts);

fp=fopen('E:\code\devkit_raw_data\devkit\pose_oxts/pose_oxts.txt','wt');
for index=1:size(pose,2)
    for index_rows =1:3
        for index_cols=1:4
            fprintf(fp,'%d',pose{1,index}(index_rows,index_cols));
            if(index_rows*index_cols<12)
                fprintf(fp,'%c',' ');
            end
        end
    end
    fprintf(fp,'\n');
end