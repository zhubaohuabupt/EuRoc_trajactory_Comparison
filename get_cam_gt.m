%���ߣ��챣��
%%%%%%%%%%%%%%%%%%%%%%%%%%%% �������ܣ��Ѹ���imu�������GTת������������0ʱ�̵�GT�����֡�����λ��%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %  ���룺imu�������GT(gt_imutoworld)���ݡ�     ---------data.csv
 %  �����
 %   �������������0ʱ�̵ı仯��
 %   <1>λ��trans��
 %   <2>��̬Rotation,��
 %   �����֡��λ�ñ仯(t+1�����tʱ��)
 %   <1>deta_trans�ġ�
 %   <2>֡����̬�仯deta_R��
 %   <3>�Լ�deta_trans��deta_R���档   
 %   ע��λ�õ�λ���ף���̬Rotation��RPY�Ǳ�������λ����
function [trans,Rotation,deta_trans,deta_R,deta_trans_inv,deta_R_inv]=get_cam_gt(gt_imutoworld,rows)

 
  T_imu_cam=[0.016350092, -0.99980514, 0.011061917, -0.021640145;
   0.99971676, 0.016537997, 0.017113992, -0.064676987;
   -0.017293599, 0.010778969, 0.99979235, 0.0098107306;
    0.0, 0.0, 0.0, 1.0];        %%%%%%%%%T_imu_cam�������imu�ı任����
T_cam_imu=zeros(4,4); 
T_cam_imu=inv(T_imu_cam);                  
T_w_imu=zeros(4,4);                  %%%%%%%%%%T_w_imu��imu������ĳ�ʼ����
T_w_imu(1,1)=1-2*gt_imutoworld(1,7)*gt_imutoworld(1,7)-2*gt_imutoworld(1,8)*gt_imutoworld(1,8);
T_w_imu(2,1)=2*gt_imutoworld(1,6)*gt_imutoworld(1,7)+2*gt_imutoworld(1,5)*gt_imutoworld(1,8);
T_w_imu(3,1)=2*gt_imutoworld(1,6)*gt_imutoworld(1,8)-2*gt_imutoworld(1,5)*gt_imutoworld(1,7);


T_w_imu(1,2)=2*gt_imutoworld(1,6)*gt_imutoworld(1,7)-2*gt_imutoworld(1,5)*gt_imutoworld(1,8);
T_w_imu(2,2)=1-2*gt_imutoworld(1,6)*gt_imutoworld(1,6)-2*gt_imutoworld(1,8)*gt_imutoworld(1,8);
T_w_imu(3,2)=2*gt_imutoworld(1,8)*gt_imutoworld(1,7)+2*gt_imutoworld(1,5)*gt_imutoworld(1,6);
 
T_w_imu(1,3)=2*gt_imutoworld(1,6)*gt_imutoworld(1,8)+2*gt_imutoworld(1,5)*gt_imutoworld(1,7);
T_w_imu(2,3)=2*gt_imutoworld(1,7)*gt_imutoworld(1,8)-2*gt_imutoworld(1,5)*gt_imutoworld(1,6);
T_w_imu(3,3)=1-2*gt_imutoworld(1,6)*gt_imutoworld(1,6)-2*gt_imutoworld(1,7)*gt_imutoworld(1,7);

T_w_imu(1,4)=gt_imutoworld(1,2);
T_w_imu(2,4)=gt_imutoworld(1,3);
T_w_imu(3,4)=gt_imutoworld(1,4);

T_w_imu(4,1)=0.0;
T_w_imu(4,2)=0.0;
T_w_imu(4,3)=0.0;
T_w_imu(4,4)=1.0;
T_imu_w=zeros(4,4); 
T_imu_w=inv(T_w_imu);
trans=zeros(rows,3);
z2=zeros(rows,6);
last_frame_T=eye(4,4);
DETAt=zeros(4,4);
deta_R=zeros(rows,3);
Rotation=zeros(rows,3);
deta_trans=zeros(rows,3);

for i=1:1:rows
    
    z2(i,1)=gt_imutoworld(10*i-9,2);
    z2(i,2)=gt_imutoworld(10*i-9,3);
    z2(i,3)=gt_imutoworld(10*i-9,4);
    z2(i,4)=gt_imutoworld(10*i-9,5);
    z2(i,5)=gt_imutoworld(10*i-9,6);
    z2(i,6)=gt_imutoworld(10*i-9,7);
    z2(i,7)=gt_imutoworld(10*i-9,8);
    w=z2(i,4);
    x=z2(i,5);
    y=z2(i,6);
    z=z2(i,7);
T_w_imu_dynamic=zeros(4,4); 
T_cam0_cam_GT=zeros(4,4); 

T_w_imu_dynamic(1,1)=1-2*y*y-2*z*z;
T_w_imu_dynamic(2,1)=2*x*y+2*w*z;
T_w_imu_dynamic(3,1)=2*x*z-2*w*y;


T_w_imu_dynamic(1,2)=2*x*y-2*w*z;
T_w_imu_dynamic(2,2)=1-2*x*x-2*z*z;
T_w_imu_dynamic(3,2)=2*z*y+2*w*x;
 
T_w_imu_dynamic(1,3)=2*x*z+2*w*y;
T_w_imu_dynamic(2,3)=2*y*z-2*w*x;
T_w_imu_dynamic(3,3)=1-2*x*x-2*y*y;

T_w_imu_dynamic(1,4)=z2(i,1);
T_w_imu_dynamic(2,4)=z2(i,2);
T_w_imu_dynamic(3,4)=z2(i,3);

T_w_imu_dynamic(4,1)=0.0;
T_w_imu_dynamic(4,2)=0.0;
T_w_imu_dynamic(4,3)=0.0;
T_w_imu_dynamic(4,4)=1.0;

T_cam0_cam_GT=T_cam_imu*T_imu_w*T_w_imu_dynamic*T_imu_cam;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%����֡������begin%%%%%%%%%%%%%%%%%%%
if i>1        %%��һ֡���� ��Ϊ��һ֡T_cam0_cam_GT�ǵ�λ��
DETAt=inv(last_frame_T)*(T_cam0_cam_GT);
%%%%%%%%%%%%%%%%%%%%%%%%����֡��RPY��
[r,p,y]=getRPY(DETAt);
deta_R(i-1,1)=r;
deta_R(i-1,2)=p;
deta_R(i-1,3)=y;
%%%%%%%%%%%%%%%%%%%%%%%%����֡��λ��
deta_trans(i-1,1)=DETAt(1,4);
deta_trans(i-1,2)=DETAt(2,4);
deta_trans(i-1,3)=DETAt(3,4);

%����detaT_t_t-1��λ��
DETAt_inv=inv(DETAt);
deta_trans_inv(i-1,1)=DETAt_inv(1,4);
deta_trans_inv(i-1,2)=DETAt_inv(2,4);
deta_trans_inv(i-1,3)=DETAt_inv(3,4);
deta_R_inv(i-1,1)=DETAt_inv(1,1);
deta_R_inv(i-1,2)=DETAt_inv(1,2);
deta_R_inv(i-1,3)=DETAt_inv(1,3);
deta_R_inv(i-1,4)=DETAt_inv(2,1);
deta_R_inv(i-1,5)=DETAt_inv(2,2);
deta_R_inv(i-1,6)=DETAt_inv(2,3);
deta_R_inv(i-1,7)=DETAt_inv(3,1);
deta_R_inv(i-1,8)=DETAt_inv(3,2);
deta_R_inv(i-1,9)=DETAt_inv(3,3);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%����֡������end%%%%%%%%%%%%%%%%%%%%%%5
last_frame_T=T_cam0_cam_GT;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%����������0ʱ�̵�λ��begin%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%������������0ʱ�̵�RPY��%%%%%%%%%%%%%%%%%

[wr,wp,wy]=getRPY(last_frame_T);

Rotation(i,1)=wr;
Rotation(i,2)=wp;
Rotation(i,3)=wy;

%%%%%%%%%%%%%%%%%%%%%%%������������0ʱ�̵�λ��%%%%%%%%%%%%%%%%%
trans(i,1)=T_cam0_cam_GT(1,4);
trans(i,2)=T_cam0_cam_GT(2,4);
trans(i,3)=T_cam0_cam_GT(3,4);
trans(i,4)=T_cam0_cam_GT(4,4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%����������0ʱ�̵�λ��end%%%%%%%%%%%%%%%%%%% 
end

