clear;
clc;
close all;

imdata = imread('Figure.jpg');%Read Image file and transform it to Matrix 
MatrixImage=imdata(:,:,1);

col=1;
%Extract all black points from the image and ignore the white ones
for i=1:size(MatrixImage,1)%Scan Matrix up to down
    for j=1:size(MatrixImage,2)% Row outer loop and Column inner one
    if MatrixImage(i,j)<=200%255=White 0=Black
        XNotMapped(:,col)=i;
        YNotMapped(:,col)=j;
        col=col+1;
    end
    end
end
XNotMappedGlobal=XNotMapped;%Store the variables 
YNotMappedGlobal=YNotMapped;

%Map X and Y using an efficient algorithm based on distance 
c=1;
Newi=1;%Since I am not able to change indiced of for loop
for i=1:size(XNotMapped,2)
    for j=1:size(XNotMapped,2)
    MinDis=sqrt((XNotMapped(j)-XNotMapped(Newi))^2 +(YNotMapped(j)-YNotMapped(Newi))^2);
    indexArray(j)=MinDis;
    end
    index=find(indexArray==1);%Priority Distance =1
    Lift=89;
    if(isempty(index)==1)%Second Priority Distance=sqrt(2)
    index=find(indexArray==sqrt(2));
    Lift=89;
    end
    if(isempty(index)==1)%Third Priority nearest point 
    index=find(indexArray<100 & indexArray>0);
    Lift=0;
    end
    if i~=size(XNotMapped,2)%No more points to map 
    index=index(1);%Since matlab is storing them in a vector
    end
    XNotMapped(Newi)=10000;
    YNotMapped(Newi)=10000;
    IndexArrayMapped(:,c)=Newi; 
    Angle3(:,c)=Lift;
    c=c+1;
    Newi=index;
end

%Construct a matrix based on the mapped indexes
for i=1:size(IndexArrayMapped,2)
XMapped(:,i)=XNotMappedGlobal(IndexArrayMapped(i));
YMapped(:,i)=YNotMappedGlobal(IndexArrayMapped(i));
end

%Apply Inverse Kinematics Equations
Angle2=zeros(1,size(XMapped,2));%Preallocate for a faster computation
Angle1=zeros(1,size(XMapped,2));
for i=1:size(XMapped,2)
D=(XMapped(i)^2+YMapped(i)^2-155^2-217^2)/(2*155*217);
Angle2(:,i)=rad2deg(atan2(sqrt(1-D^2),D));
A=217*sind(Angle2(i));
B=155+(217*cosd(Angle2(i)));
Angle1(:,i)=rad2deg(atan2(YMapped(i),XMapped(i))-atan2(A,B));
end

%Double check if Computation is correct
XCheck=155*cosd(Angle1)+217*cosd(Angle1+Angle2);
YCheck=155*sind(Angle1)+217*sind(Angle1+Angle2);

%Display some results 
display(XMapped);
display(YMapped);
display(Angle1);
display(Angle2);
display(Angle3);
display(XCheck);
display(YCheck);

%Send Data to Vrep and Start Drawing
VrepDrawImage(Angle1,Angle2,Angle3);