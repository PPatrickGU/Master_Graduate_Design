clc; close all;

GridSize = 30;  %length of the square
m =7;      %number of row
n = 10;       % number of col
margin = 10; % white boarder size
I = ones(m*GridSize+2*margin,n*GridSize+2*margin)*255;

%the first grid is black
for i = 1:m
    if mod(i,2)==1
        for j = 1:n
            if mod(j,2)==1
                I(1+(i-1)*GridSize+margin:i*GridSize+margin,...
                1+(j-1)*GridSize+margin:j*GridSize+margin) = 0;
            end
        end
    else
        for j = 1:n
            if mod(j,2)==0
               I(1+(i-1)*GridSize+margin:i*GridSize+margin,...
                1+(j-1)*GridSize+margin:j*GridSize+margin) = 0;
            end
        end
    end
end
imshow(I);

imwrite(I,'chessboard.ppm');
