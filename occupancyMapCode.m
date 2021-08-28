clc
clear all

% define map size
map = binaryOccupancyMap(10,10,2)

% define objects inside the map
x = [3;3.5;4;4.5;5;5.5;6;6.5;7;7.5; 3;3.5;4;4.5;5;5.5;6;6.5;7;7.5; 3;3.5;4;4.5;5;5.5;6;6.5;7;7.5]
y = [3;3;3;3;3;3;3;3;3;3; 5.5;5.5;5.5;5.5;5.5;5.5;5.5;5.5;5.5;5.5; 8;8;8;8;8;8;8;8;8;8]

% create binary occupancy map
setOccupancy(map, [x y], ones(30,1))
figure
show(map)

% inflate objects in map for better navigation
robotRadius = 0.5;
mapInflated = copy(map);
inflate(mapInflated,robotRadius);
show(mapInflated)

% apply prm algorithm to map
prm = mobileRobotPRM;
prm.Map = mapInflated;
prm.NumNodes = 100;
prm.ConnectionDistance = 5;

% define start and end locations
startLocation = [1 1];
endLocation = [9 9];    

% prm defines path
path = findpath(prm, startLocation, endLocation)
show(prm)

% waypoint counter
waypointNumber = size(path,1)

% assign x,y of waypoints according to waypoint number
if waypointNumber == 5
p1 = path(1,:);
p2 = path(2,:);
p3 = path(3,:);
p4 = path(4,:);
p5 = path(5,:);

x1 = p1(1);
y1 = p1(2);

x2 = p2(1);
y2 = p2(2);

x3 = p3(1);
y3 = p3(2);

x4 = p4(1);
y4 = p4(2);

x5 = p5(1);
y5 = p5(2);
 
x1 = round(x1,2);
y1 = round(y1,2);

x2 = round(x2,2);
y2 = round(y2,2);

x3 = round(x3,2);
y3 = round(y3,2);

x4 = round(x4,2);
y4 = round(y4,2);

x5 = round(x5,2);
y5 = round(y5,2);

vely_1 = (y2-y1);
velx_1 = (x2-x1);

vely_2 = (y3-y2);
velx_2 = (x3-x2);

vely_3 = (y4-y3);
velx_3 = (x4-x3);

vely_4 = (y5-y4);
velx_4 = (x5-x4);
end

if waypointNumber == 6
p1 = path(1,:);
p2 = path(2,:);
p3 = path(3,:);
p4 = path(4,:);
p5 = path(5,:);
p6 = path(6,:);

x1 = p1(1);
y1 = p1(2);

x2 = p2(1);
y2 = p2(2);

x3 = p3(1);
y3 = p3(2);

x4 = p4(1);
y4 = p4(2);

x5 = p5(1);
y5 = p5(2);

x6 = p6(1);
y6 = p6(2);

x1 = round(x1,2);
y1 = round(y1,2);

x2 = round(x2,2);
y2 = round(y2,2);

x3 = round(x3,2);
y3 = round(y3,2);

x4 = round(x4,2);
y4 = round(y4,2);

x5 = round(x5,2);
y5 = round(y5,2);

x6 = round(x6,2);
y6 = round(y6,2);

vely_1 = (y2-y1);
velx_1 = (x2-x1);

vely_2 = (y3-y2);
velx_2 = (x3-x2);

vely_3 = (y4-y3);
velx_3 = (x4-x3);

vely_4 = (y5-y4);
velx_4 = (x5-x4);

vely_5 = (y6-y5);
velx_5 = (x6-x5);
end
    
if waypointNumber == 7
p1 = path(1,:);
p2 = path(2,:);
p3 = path(3,:);
p4 = path(4,:);
p5 = path(5,:);
p6 = path(6,:);
p7 = path(7,:);

x1 = p1(1);
y1 = p1(2);

x2 = p2(1);
y2 = p2(2);

x3 = p3(1);
y3 = p3(2);

x4 = p4(1);
y4 = p4(2);

x5 = p5(1);
y5 = p5(2);

x6 = p6(1);
y6 = p6(2);

x7 = p7(1);
y7 = p7(2);

x1 = round(x1,2);
y1 = round(y1,2);

x2 = round(x2,2);
y2 = round(y2,2);

x3 = round(x3,2);
y3 = round(y3,2);

x4 = round(x4,2);
y4 = round(y4,2);

x5 = round(x5,2);
y5 = round(y5,2);

x6 = round(x6,2);
y6 = round(y6,2);

x7 = round(x7,2);
y7 = round(y7,2);

vely_1 = (y2-y1);
velx_1 = (x2-x1);

vely_2 = (y3-y2);
velx_2 = (x3-x2);

vely_3 = (y4-y3);
velx_3 = (x4-x3);

vely_4 = (y5-y4);
velx_4 = (x5-x4);

vely_5 = (y6-y5);
velx_5 = (x6-x5);

vely_6 = (y7-y6);
velx_6 = (x7-x6);
end

if waypointNumber == 8
p1 = path(1,:);
p2 = path(2,:);
p3 = path(3,:);
p4 = path(4,:);
p5 = path(5,:);
p6 = path(6,:);
p7 = path(7,:);
p8 = path(8,:);

x1 = p1(1);
y1 = p1(2);

x2 = p2(1);
y2 = p2(2);

x3 = p3(1);
y3 = p3(2);

x4 = p4(1);
y4 = p4(2);

x5 = p5(1);
y5 = p5(2);

x6 = p6(1);
y6 = p6(2);

x7 = p7(1);
y7 = p7(2);

x8 = p8(1);
y8 = p8(2);

x1 = round(x1,2);
y1 = round(y1,2);

x2 = round(x2,2);
y2 = round(y2,2);

x3 = round(x3,2);
y3 = round(y3,2);

x4 = round(x4,2);
y4 = round(y4,2);

x5 = round(x5,2);
y5 = round(y5,2);

x6 = round(x6,2);
y6 = round(y6,2);

x7 = round(x7,2);
y7 = round(y7,2);

x8 = round(x8,2);
y8 = round(y8,2);

vely_1 = (y2-y1);
velx_1 = (x2-x1);

vely_2 = (y3-y2);
velx_2 = (x3-x2);

vely_3 = (y4-y3);
velx_3 = (x4-x3);

vely_4 = (y5-y4);
velx_4 = (x5-x4);

vely_5 = (y6-y5);
velx_5 = (x6-x5);

vely_6 = (y7-y6);
velx_6 = (x7-x6);

vely_7 = (y8-y7);
velx_7 = (x8-x7);
end