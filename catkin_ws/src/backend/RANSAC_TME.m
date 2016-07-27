
close all;
oldDepth = imread('Frames/dep_15.png');
oldFrame = imread('Frames/rgb_15.png');
newDepth = imread('Frames/dep_20.png');
newFrame = imread('Frames/rgb_20.png');
% readKey(); 
% readKeyNew(); 

% newMatch = [ 0, 1, 2, 5, 7, 8, 10, 9, 13, 14, 11, 17, 19, 16, 15, 23,14, 27, 18, 25, 32, 34, 35, 36, 37, 38, 42, 48, 45, 47 ];
% oldMatch = [ 0, 1, 2, 3, 6, 8, 9, 10, 11, 12, 14, 15, 16, 17, 19, 22, 24, 25, 26, 27, 30, 32, 33, 36, 37, 38, 42, 43, 45, 47 ];

oldMatch = [ 0, 1, 3, 6, 8, 9, 10, 11, 14, 13, 12, 16, 15, 15, 20, 17, 22, 21, 25, 27, 30, 29, 32, 34, 35, 36, 38, 37, 39, 44 ];
newMatch = [ 0, 1, 4, 5, 8, 9, 10, 13, 14, 15, 16, 17, 18, 20, 22, 23, 26, 27, 30, 31, 33, 34, 36, 37, 38, 39, 40, 41, 42, 43 ];


newMatch = newMatch + 1; 
oldMatch = oldMatch + 1;

KeyOld=[floor(Key15x(oldMatch)),floor(Key15y(oldMatch))];

KeyNew=[floor(Key20x(newMatch)),floor(Key20y(newMatch))];

imshow(newFrame); hold on; plot(KeyNew(:,1),KeyNew(:,2),'bo');
figure
pause(0.1)
imshow(oldFrame); hold on; plot(KeyOld(:,1),KeyOld(:,2),'ro');
figure
pause(0.1)
showMatchedFeatures(newFrame,oldFrame,KeyNew,KeyOld,'montage');

% showMatchedFeatures(newFrame,oldFrame,KeyNew,KeyOld);
% 


