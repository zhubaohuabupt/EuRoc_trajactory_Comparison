%作者：朱保华
%%%%%%%%%%%%%%%%%%%%%%%%%%%% 函数功能：把给的imu到世界的GT转化至相机到相机0时刻的GT并输出帧间相对位姿%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %  输入：imu到世界的GT(gt_imutoworld)数据。     ---------data.csv
 %  输出：
 %   相机的相对于相机0时刻的变化量
 %   <1>位置trans、
 %   <2>姿态Rotation,、
 %   相机的帧间位置变化(t+1相对于t时刻)
 %   <1>deta_trans的、
 %   <2>帧间姿态变化deta_R、
 %   <3>以及deta_trans和deta_R的逆。   
 %   注：位置单位：米，姿态Rotation用RPY角表征，单位：度
function [trans,Rotation,deta_trans,deta_R,deta_trans_inv,deta_R_inv]=get_cam_gt(gt_imutoworld,rows)

 
  T_imu_cam=[0.016350092, -0.99980514, 0.011061917, -0.021640145;
   0.99971676, 0.016537997, 0.017113992, -0.064676987;
   -0.017293599, 0.010778969, 0.99979235, 0.0098107306;
    0.0, 0.0, 0.0, 1.0];        %%%%%%%%%T_imu_cam是相机到imu的变换矩阵
T_cam_imu=zeros(4,4); 
T_cam_imu=inv(T_imu_cam);                  
T_w_imu=zeros(4,4);                  %%%%%%%%%%T_w_imu是imu到世界的初始矩阵
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%计算帧间增量begin%%%%%%%%%%%%%%%%%%%
if i>1        %%第一帧不用 因为第一帧T_cam0_cam_GT是单位阵
DETAt=inv(last_frame_T)*(T_cam0_cam_GT);
%%%%%%%%%%%%%%%%%%%%%%%%计算帧间RPY角
[r,p,y]=getRPY(DETAt);
deta_R(i-1,1)=r;
deta_R(i-1,2)=p;
deta_R(i-1,3)=y;
%%%%%%%%%%%%%%%%%%%%%%%%计算帧间位置
deta_trans(i-1,1)=DETAt(1,4);
deta_trans(i-1,2)=DETAt(2,4);
deta_trans(i-1,3)=DETAt(3,4);

%存入detaT_t_t-1的位姿
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%计算帧间增量end%%%%%%%%%%%%%%%%%%%%%%5
last_frame_T=T_cam0_cam_GT;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%计算相对相机0时刻的位姿begin%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%计算相对于相机0时刻的RPY角%%%%%%%%%%%%%%%%%

[wr,wp,wy]=getRPY(last_frame_T);

Rotation(i,1)=wr;
Rotation(i,2)=wp;
Rotation(i,3)=wy;

%%%%%%%%%%%%%%%%%%%%%%%计算相对于相机0时刻的位置%%%%%%%%%%%%%%%%%
trans(i,1)=T_cam0_cam_GT(1,4);
trans(i,2)=T_cam0_cam_GT(2,4);
trans(i,3)=T_cam0_cam_GT(3,4);
trans(i,4)=T_cam0_cam_GT(4,4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%计算相对相机0时刻的位姿end%%%%%%%%%%%%%%%%%%% 
end

