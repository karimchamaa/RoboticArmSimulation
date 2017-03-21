clear;%Clear the screen, delete the variables and close all plots.
close all;
clc;

StepSize=361;% Step size between min and max
q = sym('q%d', [2 1]);%Symbols for joint variables 
Index=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];

% DH-Table
table=[155 0 0 q(1)
       217 0 0 q(2)];
%Constraints Joints 1 and 2
Min=[0 0];
Max=[180 170];

% Get A1,A2
A=sym('a',[4,4,size(table,1)]);%Preallocate for faster computation
for i=1:size(table,1)
A(:,:,i)=[cos(table(i,4)) -sin(table(i,4))*cos(table(i,2)) sin(table(i,4))*sin(table(i,2)) table(i,1)*cos(table(i,4))
sin(table(i,4)) cos(table(i,4))*cos(table(i,2)) -cos(table(i,4))*sin(table(i,2)) table(i,1)*sin(table(i,4))
0 sin(table(i,2)) cos(table(i,2)) table(i,3)
0 0 0 1];%DH Matrix
end

%Homogenous Transform Matrix
ZBase=sym('zbase',[4,4,size(table,1)]);%Preallocate for faster computation
 for i=1:size(table,1)
      H=Index*A(:,:,i);
      Index=H;
      ZBase(:,:,i)=H;%Will be used by Jacobian 
      X=H(1,4);%store position
      Y=H(2,4);
 end 
 
 %Jacobians
J=jacobian([H(1,4),H(2,4)],[q(1),q(2)]);

 %Equate Vectors, size of vector depend on step size
 Q=zeros(size(table,1),StepSize);%Preallocate for faster computation
 for i=1:size(table,1)%Convert angle to radiant 
      Q(i,:)=deg2rad(linspace(Min(i),Max(i),StepSize));
 end

% Find all the possible Combinations 
Q=combvec(Q(1,:),Q(2,:));%Combine joint variables  
for i=1:size(Q,1)
eval(['q' num2str(i) '= Q(i,:);']);%Store Combined parameter in q1...qn
end

%Plot Vectors of same size in 2d
pl1=plot(eval(X),eval(Y),'b.'); 
grid on;
axis equal;
xlabel('x-axis');%Label the axix. 
ylabel('y-axis');
title('Reachable and Singular Workspace');

%Singular Workspace Generation 
for i=1:size(table,1) % Make q1..qn as symbolic 
syms (['q' num2str(i)'])
end
s = sym('s%d', [1000 1]);%Create a singularity vector s composed of symbols 
syms zero;% which will be used in solving with respect to 0 to get 2 angles

DetJ=simplify(det(J));%Determinant Jacobian

if (DetJ~=0)% Meaning no singluar points and do not proceed with code
for i=1:size(table,1)
 Var=eval(q(i));
 temp=solve(DetJ==zero,Var);%Solve for each joint variable 
 eval(['s' num2str(i) '= temp;']);%Store result in a singular vector
end
zero=0;% Give zero it's value and evaluate the singularities

for i=1:size(table,1)%Outer Loop representing the number of Joint Variables
 if(size(eval(s(i)),1)~=0)%If the vector is not empty meaning Singular pt
  for j=1:size(eval(s(i)),1)%Number of singularities in a joint variable
       temp=eval(s(i));
       temp=eval(temp);
       if(temp(j)<0)% Scale negative Rad angle to pi-2*pi
           temp(j)=(2*pi)-(-temp(j));
       end
       for k=1:size(table,1)%Replace other joint variables by their values
       if(i~=k)
          eval(['q' num2str(k) '= Q(k,:);']);
       end
       end
       %Check if joint variable is inside the range of Min-Max
       if( temp(j)>=deg2rad(Min(i)) && temp(j)<=deg2rad(Max(i)) )
           eval(['q' num2str(i) '= linspace(temp(j),temp(j),size(Q,2));']);     
       hold on;% Plot in 2D for each singular values and hold on the plot
       pl2=plot(eval(X),eval(Y),'r.');
       end
  end
 end
end
end
legend([pl1 pl2],{'Reachable','Singular'});% Legend for plots
axis equal;
