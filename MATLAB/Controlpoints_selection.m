ra_base = 'Outdoor_facility\DOP20_utm32_681500_5406000_500';
of_base = 'Roundabout\DOP20_utm32_682000_5406000_500';
[moving,cmap1] = imread(strcat(ra_base,'.tif'));
R1 = worldfileread(strcat(ra_base,'.tfw'),'planar',size(moving));
[fixed ,cmap2] = imread(strcat(of_base,'.tif'));
R2 = worldfileread(strcat(of_base,'.tfw'),'planar',size(fixed));
cpselect(moving, fixed);