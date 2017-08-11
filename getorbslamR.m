%���ߣ��챣��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%�������ܣ���orbslam�������Ԫ����̬����ת����RPY��%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [rpydata]=getorbslamR (CameraTrajectory_txt)
[rownum colnum]=size(CameraTrajectory_txt);
rpydata=zeros(rownum,3);


for i=1:1:rownum
    x=CameraTrajectory_txt(i,5);
    y=CameraTrajectory_txt(i,6);
    z=CameraTrajectory_txt(i,7);
     w=CameraTrajectory_txt(i,8);
    [rr,pp,yy]= QuaternionToRPY(x,y,z,w);
   
    
       rpydata(i,1)=rr;
       rpydata(i,2)=pp;
         rpydata(i,3)=yy;

end