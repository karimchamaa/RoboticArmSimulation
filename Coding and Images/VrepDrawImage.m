% V-rep Part Synchronous Communication with Matlab 
function VrepDrawImage(Angle1,Angle2,Angle3)
	disp('Program started');
	% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
	vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
	vrep.simxFinish(-1); % just in case, close all opened connections
	clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
	if (clientID>-1)
		disp('Connected to remote API server');
		% enable the synchronous mode on the client:
		vrep.simxSynchronous(clientID,true);
		% start the simulation:
		vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
		% Now step a few times:
	% getting handles and position for everything
%Get object handles
for i=0:1    
[~,handlePen]=vrep.simxGetObjectHandle(clientID,'Pencil', vrep.simx_opmode_blocking);
[~,handleJoint1]=vrep.simxGetObjectHandle(clientID,'Joint1', vrep.simx_opmode_blocking);
[~,handleJoint2]=vrep.simxGetObjectHandle(clientID,'Joint2', vrep.simx_opmode_blocking);
[~,handleJoint3]=vrep.simxGetObjectHandle(clientID,'Joint3', vrep.simx_opmode_blocking);
end
%Send Angles in degrees to V-Rep
for i=1:size(Angle1,2)
 vrep.simxSetJointPosition(clientID,handleJoint1,deg2rad(Angle1(i)),vrep.simx_opmode_oneshot);
 vrep.simxSetJointPosition(clientID,handleJoint2,deg2rad(Angle2(i)),vrep.simx_opmode_oneshot);
 if(i==1 || Angle3(i-1)==0)%Pause in order to reach angle before lifting
 pause(1);
 end
 vrep.simxSetJointPosition(clientID,handleJoint3,deg2rad(-(Angle3(i)+1)),vrep.simx_opmode_oneshot);
  if(i==1 || Angle3(i-1)==0)%Pause in order to lift or bring down the pen
 pause(1);
  end 
pause(0.2)%Pause between Angles
if(Angle3(i)==89)%Read only if Pen is down with repect to revolute joint 1
[~,positionLink3]=vrep.simxGetObjectPosition(clientID,handlePen,handleJoint1,vrep.simx_opmode_streaming);
positionLink3=[positionLink3(1),positionLink3(2)];%Display X and Y
disp(positionLink3)
plot(positionLink3(1),positionLink3(2),'.b-');
axis equal;
hold on;
grid on;
xlabel('x-axis');%Label the axix. 
ylabel('y-axis');
title('V-REP Drawing');
axis([0 0.2 0 0.31]);%Upper limits of workspace 
end
end
		% stop the simulation:
		vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
		% Now close the connection to V-REP:	
		vrep.simxFinish(clientID);
	else
		disp('Failed connecting to remote API server');
	end
	vrep.delete(); % call the destructor!
	disp('Program ended');
end
