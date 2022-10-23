clear
clf
clc
%% IRB120
r = IRB120(transl(0,0,0));
%%
try delete(Link1ellipsoid); end;
try delete(Link2ellipsoid); end;
try delete(Link3ellipsoid); end;
try delete(Link5ellipsoid); end;
%%
q0 = r.model.getpos();
jointTransforms = {};
modelPoints = {r.model.base};
for i=1:6
    jointTransforms{i} = r.model.A(i, q0);
    modelPoints{i+1} = cell2mat(modelPoints(i))*cell2mat(jointTransforms(i));
end

hold on;
% Base/Link 1

%%
centerPoint{1} = transl(cell2mat(modelPoints(2))*(transl(0,0.145,0))); % FIXED

radii{1} = [0.107,0.107,0.145];
[X,Y,Z] = ellipsoid( centerPoint{1}(1), centerPoint{1}(2), centerPoint{1}(3), radii{1}(1), radii{1}(2), radii{1}(3) );
Link1ellipsoid = surf(X,Y,Z);
view(3);
%%
% Link 2
centerPoint{2} = transl(cell2mat(modelPoints(3))*transl(-0.135,0,0)); %needs to be movable

radii{2} = [0.09,0.107,0.186];
[X,Y,Z] = ellipsoid( centerPoint{2}(1), centerPoint{2}(2), centerPoint{2}(3), radii{2}(1), radii{2}(2), radii{2}(3) );
Link2ellipsoid = surf(X,Y,Z);
%%
% Link 3/4
centerPoint{3} = transl(cell2mat(modelPoints(4))*transl(0,0,0.142)); %needs to be movable[0.151,0,0.639]

radii{3} = [0.231,0.09,0.07];
[X,Y,Z] = ellipsoid( centerPoint{3}(1), centerPoint{3}(2), centerPoint{3}(3), radii{3}(1), radii{3}(2), radii{3}(3) );
Link3ellipsoid = surf(X,Y,Z);

%%
% Link 5/6
centerPoint{4} = transl(cell2mat(modelPoints(6))*transl(0,0,-0.08)); %%[0.305,0,0.639];
radii{4} = [0.041,0.041,0.1475];
[X,Y,Z] = ellipsoid( centerPoint{4}(1), centerPoint{4}(2), centerPoint{4}(3), radii{4}(1), radii{4}(2), radii{4}(3) );
Link5ellipsoid = surf(X,Y,Z);
axis equal


%% generate obsticle @ 0.3,-0.4,0
% centerPoint = [0.3,-0.45,0.5];
% [Y,Z] = meshgrid(-0.45:0.05:-0.35,0:0.05:0.1);
[Y,Z] = meshgrid(-0.1:0.05:0.1,-0.1:0.05:0.1);
sizeMat = size(Y);
X = repmat(0.1,sizeMat(1),sizeMat(2));

cubePoints = [X(:),Y(:),Z(:)];
cubePoints = [ cubePoints ...
             ; cubePoints * rotz(pi/2)...
             ; cubePoints * rotz(pi) ...
             ; cubePoints * rotz(3*pi/2) ...
             ; cubePoints * roty(pi/2) ...
             ; cubePoints * roty(-pi/2)];  

cubePoints = cubePoints + repmat([0.3,-0.4,0.1],size(cubePoints,1),1);
cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
%% Check Collision

for i = 1:4
    cP = centerPoint{i};
    rad = radii{i};
    algebraicDist = GetAlgebraicDist(cubePoints, cP, rad);
    pointInside = find(algebraicDist<1);
    display(['COLLISION ABORT']);
end

function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
              + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end