close all;
clear all;

oldID = 15;
newID = 16;
filename = '../Frames/dep_%d.png';
filename = sprintf(filename,oldID);
oldDepth = imread(filename);

filename = '../Frames/rgb_%d.png';
filename = sprintf(filename,oldID);
oldFrame = imread(filename);

filename = '../Frames/dep_%d.png';
filename = sprintf(filename,newID);
newDepth = imread(filename);

filename = '../Frames/rgb_%d.png';
filename = sprintf(filename,newID);
newFrame = imread(filename);



% readKey(); 
raw_text = fileread('consensus.txt');
for i = 1:length(raw_text)
    if (raw_text(i) == '[')
        raw_text = raw_text(i+1:end);
    end
    
   if (raw_text(i) == ']')
        raw_text = raw_text(1:i-1);
        break;
    end
end

fileID = fopen('consensus.txt','w');
fprintf(fileID,raw_text);
consensus = importdata('consensus.txt');

consensus = consensus + 1; 
[row, col] = size(consensus);
consensus = reshape(consensus', [row*col,1]);
consensus = consensus(consensus<2000);

% readKeyNew(); 
raw_text = fileread('match1.txt');
for i = 1:length(raw_text)
    if (raw_text(i) == '[')
        raw_text = raw_text(i+1:end);
    end
    
    if (raw_text(i) == ']')
        raw_text = raw_text(1:i-1);
        break;
    end
end

fileID = fopen('match1.txt','w');
fprintf(fileID,raw_text);
newMatch = importdata('match1.txt');

% readKeyOld(); 
raw_text = fileread('match2.txt');
for i = 1:length(raw_text)
    if (raw_text(i) == '[')
%         oldID = str2double(raw_text(i-4))+3;
%         newID = oldID+1;
        raw_text = raw_text(i+1:end);
    end
    
    if (raw_text(i) == ']')
        raw_text = raw_text(1:i-1);
        break;
    end
    
end

fileID = fopen('match2.txt','w');
fprintf(fileID,raw_text);
oldMatch = importdata('match2.txt');

newMatch = newMatch + 1; 
[row, col] = size(newMatch);
newMatch = reshape(newMatch', [row*col,1]);

for i= 1:length(newMatch)
    newMatch = newMatch(newMatch < 2000);
end

oldMatch = oldMatch + 1; % finished reading matches
[row, col] = size(oldMatch);
oldMatch = reshape(oldMatch', [row*col,1]);

for i= 1:length(oldMatch)
    oldMatch = oldMatch(oldMatch < 2000);
end

raw_text = fileread('keypoints3D.csv');
for i = 1:length(raw_text)
    if (raw_text(i) == '`')
        if(str2num(raw_text(i+1:i+2)) == oldID)
            start_file = i;
        end
        
        if(str2num(raw_text(i+1:i+2))== newID)
            mid_file = i-16;
        end
        
        
        if(str2num(raw_text(i+1:i+2))== newID+1)
            end_file = i-16;
        end
        
    end
end
% raw_text = raw_text(start_file:mid_file);
fileID = fopen('old_keypoint.csv','w');
fprintf(fileID,raw_text(start_file:mid_file));

fileID = fopen('new_keypoint.csv','w');
fprintf(fileID,raw_text(mid_file:end_file));

newKey = importdata('new_keypoint.csv');
oldKey = importdata('old_keypoint.csv');

if isstruct(newKey)
    newKey  = newKey.data;
end

if isstruct(oldKey)
    oldKey  = oldKey.data;
end

KeyOld= oldKey(oldMatch,:); %final assignment
KeyNew= newKey(newMatch,:);

raw_text = fileread('keypoints.csv');
for i = 1:length(raw_text)
    if (raw_text(i) == '`')
        if(str2num(raw_text(i+1:i+2)) == oldID)
            start_file = i;
        end
        
        if(str2num(raw_text(i+1:i+2))== newID)
            mid_file = i-14;
        end
        
        
        if(str2num(raw_text(i+1:i+2))== newID+1)
            end_file = i-16;
        end
        
    end
end
% raw_text = raw_text(start_file:mid_file);
fileID = fopen('old_keypoint.csv','w');
fprintf(fileID,raw_text(start_file:mid_file));

fileID = fopen('new_keypoint.csv','w');
fprintf(fileID,raw_text(mid_file:end_file));

newKey = importdata('new_keypoint.csv');
oldKey = importdata('old_keypoint.csv');

if isstruct(newKey)
    newKey  = newKey.data;
end

if isstruct(oldKey)
    oldKey  = oldKey.data;
end

keyOldPix = oldKey(oldMatch,:); %final assignment
keyNewPix = newKey(newMatch,:);

keyOldPix = keyOldPix(consensus,:);
keyNewPix = keyNewPix(consensus,:);

imshow(newFrame); hold on; plot(keyNewPix(:,1),keyNewPix(:,2),'bo');
figure
pause(0.1)
imshow(oldFrame); hold on; plot(keyOldPix(:,1),keyOldPix(:,2),'ro');
figure
pause(0.1)

showMatchedFeatures(newFrame,oldFrame,keyNewPix,keyOldPix,'montage');

% showMatchedFeatures(newFrame,oldFrame,KeyNew,KeyOld);
% 


% filtering the depth map ?
% surf(newDepth)







