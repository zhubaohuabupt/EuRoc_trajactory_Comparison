%���ߣ��챣��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%�������ܣ���Ԫ����̬����ת����RPY��%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [rr,pp,yy]=QuaternionToRPY(x,y,z,w)


yy=atan2(2*(w*x+y*z),1-2*(x*x+y*y))*180/3.14;
pp=asin(2*(w*y-z*x))*180/3.14;
rr=atan2(2*(w*z+x*y),1-2*(y*y+z*z))*180/3.14;



end