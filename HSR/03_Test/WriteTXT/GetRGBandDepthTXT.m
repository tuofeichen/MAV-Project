%% get two .txt files with the names of the used images in it


pathRGB = 'C:\Users\Michael\Desktop\out\rgb\';
pathDepth = 'C:\Users\Michael\Desktop\out\depth\';

filesRGB = dir([pathRGB '*.png']);
filesDepth = dir([pathDepth '*.png']);

nRGB = length(filesRGB);
nDepth = length(filesDepth);

fileNamesRGB = zeros(nRGB, 1);
fileNamesDepth = zeros(nDepth, 1);

if nRGB == nDepth
    
    for n = 1:nRGB
        idx = strfind(filesRGB(n).name, '.png');
        fileNamesRGB(n,1) = str2num(filesRGB(n).name(1:idx-1));
        idx = strfind(filesDepth(n).name, '.png');
        fileNamesDepth(n,1) = str2num(filesDepth(n).name(1:idx-1));
    end
else
    error('Number of Depth and RGB images is Not equal!');
end

fileNamesRGB = sort(fileNamesRGB);
fileNamesDepth = sort(fileNamesDepth);

rgbTxtFileName = 'rgb.txt';
depthTxtFileName = 'depth.txt';

fidRGB = fopen(rgbTxtFileName, 'w', 'n', 'UTF-8');
fidDepth = fopen(depthTxtFileName, 'w', 'n', 'UTF-8');

fprintf(fidRGB, '# color images\n');
fprintf(fidRGB, '# timestamp filename\n');

fprintf(fidDepth, '# depth map\n');
fprintf(fidDepth, '# timestamp filename\n');

for n=1:nRGB
    fprintf(fidRGB, '%f rgb/%f.png\n', fileNamesRGB(n,1), fileNamesRGB(n,1));
    fprintf(fidDepth, '%f depth/%f.png\n', fileNamesDepth(n,1), fileNamesDepth(n,1));
end

fclose(fidRGB);
fclose(fidDepth);
