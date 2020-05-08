%% Attempt to visualise the robot
% Define number of link
N=9;

% Calculate D-H parameter table
dhparams=zeros(2*N,4);
for i=1:N
    if rem(i,2)==0
        dhparams(2*i-1:2*i,:)=[pi pi/2 0 0; pi 0 a 0];
    else
        dhparams(2*i-1:2*i,:)=[0 pi/2 0 0; 0 0 a 0];
    end
end

% Initialise links and body
links=cell(N,1);
joints=cell(2*(N-1),1);

% Build the robot
cdhrm = rigidBodyTree;

for i=1:2*N
    parent_link_name=sprintf('link%d',i-1);
    link_name = sprintf('link%d',i);
    joint_name=sprintf('joint%d',i);
   
    links{i} = rigidBody(link_name);
    joints{i} = rigidBodyJoint(joint_name,'revolute');
 
    setFixedTransform(joints{i},dhparams(i,:),'dh');
    
    links{i}.Joint = joints{i};
    if i==1
       basename = cdhrm.BaseName;
       addBody(cdhrm,links{i},basename);
    else
       addBody(cdhrm,links{i},parent_link_name);
    end
end
showdetails(cdhrm)
show(cdhrm)