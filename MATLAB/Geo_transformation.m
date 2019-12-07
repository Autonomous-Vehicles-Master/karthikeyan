load controlpoints.mat
movingPoints_dist = pdist(movingPoints,'euclidean');
fixedPoints_dist = pdist(fixedPoints,'euclidean');
temp = fixedPoints(1,1)+(fixedPoints(2,1)-fixedPoints(1,1))*movingPoints_dist/fixedPoints_dist;
fixedPoints(2,2) = fixedPoints(1,2)+(fixedPoints(2,2)-fixedPoints(1,2))*movingPoints_dist/fixedPoints_dist;
fixedPoints(2,1) = temp;

[X1, Y1] = intrinsicToWorld(R1, movingPoints(:,1), movingPoints(:,2));
[X2, Y2] = intrinsicToWorld(R2, fixedPoints(:,1), fixedPoints(:,2));
movingPoints_wc = [X1, Y1];
fixedPoints_wc = [X2, Y2];

figure
mapshow(moving, cmap1, R1);
line(X1,Y1,'Color','w');
figure
mapshow(fixed, cmap2, R2);
line(X2,Y2,'Color','w');

tform_pc = fitgeotrans(movingPoints,fixedPoints,'NonreflectiveSimilarity');
ss_pc = tform_pc.T(2,1);
sc_pc = tform_pc.T(1,1);
scale_recovered_pc = sqrt(ss_pc*ss_pc + sc_pc*sc_pc);
theta_recovered_pc = atan2(ss_pc,sc_pc)*180/pi;

tform = fitgeotrans(movingPoints_wc,fixedPoints_wc,'NonreflectiveSimilarity');
ss = tform.T(2,1);
sc = tform.T(1,1);
scale_recovered = sqrt(ss*ss + sc*sc);
theta_recovered = atan2(ss,sc)*180/pi;

Rfixed = imref2d(size(fixed));
registered1 = imwarp(moving,tform_pc,'FillValues', 255,'OutputView',Rfixed);
figure, imshowpair(fixed,registered1,'blend');

disp("Final Transformation Matrix in UTM Coordinates is ")
format longG
disp(transpose(tform.T))

disp("Rotation is ")
format longG
disp(theta_recovered)

disp("Transformation is ")
format longG
disp(tform.T(3,1))
disp(tform.T(3,2))