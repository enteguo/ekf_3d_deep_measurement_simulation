clear
timestamps = importdata('E:\dataset\2011_09_26_drive_0009_sync\2011_09_26\2011_09_26_drive_0009_sync\oxts\timestamps.txt');
timestamps = timestamps.data;            %ʱ���
oxts = loadOxtsliteData('E:\dataset\2011_09_26_drive_0009_sync\2011_09_26\2011_09_26_drive_0009_sync')%��������Ϣ��x,y,z������ٶ��ڣ�18��19��20��


e = calcangles(oxts,timestamps);

v = velocity(oxts,timestamps,e);

p = translation(v,e,oxts,timestamps);