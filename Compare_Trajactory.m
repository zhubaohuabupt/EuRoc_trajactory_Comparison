%作者：朱保华
%这个是主函数


  %Scheme1至3是 读取orb输出的位姿，根据测试的方案个数不同，可以选择注释或添加新的方案。
 scheme1 =importdata('D:\matlab_program\compare_trajactory\202\CameraTrajectory2021.txt');%   方案1
 scheme2 =importdata('D:\matlab_program\compare_trajactory\202\CameraTrajectory202_getfts_chang.txt');%方案2
 scheme3 =importdata('D:\matlab_program\compare_trajactory\202\CameraTrajectory202_getfts_chang.txt');%方案3

 %获取输出数据的大小，以便产生相同大小的groudtruth大小的数据。
 [rows cols]=size(scheme1);
 
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%对groundtruth的处理%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %groundtruth是EvRoc图集提供的数据，是imu相对于世界坐标的位姿。
 groundtruth=csvread('F:\slam_Trajectory\Vgroundtruth\V202\data.csv',7,0);%黑       7是特殊数字
 %利用提供的groundtruth，求取相机的相对于相机0时刻的位姿以及帧间变化量，详见get_cam_gt函数注释。
   [GT_trans,GT_Rotation,GT_deta_trans,GT_deta_R,GT_deta_trans_inv,GT_deta_R_inv]=get_cam_gt(groundtruth,rows);
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%对slam输出数据的处理：%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
  %从orb输出的位姿数据中获取旋转矩阵，并去除临界突变。
   scheme1RR=getorbslamR(scheme1);
   scheme2RR=getorbslamR(scheme2);
   scheme3RR=getorbslamR(scheme3);
   scheme1R=correctRPY(scheme1RR);
   scheme2R=correctRPY(scheme2RR);
   scheme3R=correctRPY(scheme3RR);
    GT_R=correctRPY(GT_Rotation);
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%开始对比精度：%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%             
   figure 
   subplot(3,1,1)
  plot(GT_trans(:,1),'k');hold on
  plot(scheme1(:,2),'r');hold on
  plot(scheme2(:,2),'g');hold on
 plot(scheme3(:,2),'b');hold on
   title( '相对于相机0时刻的位置-----------x'); 
      xlabel('帧');
      ylabel('y/m')
subplot(3,1,2)
 plot(GT_trans(:,2),'k');hold on 
  plot(scheme1(:,3),'r');hold on
 plot(scheme2(:,3),'g');hold on
 plot(scheme3(:,3),'b');hold on
    title( '相对于相机0时刻的位置-----------y'); 
      xlabel('帧');
      ylabel('y/m');
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%相机位置对比
 subplot(3,1,3)
   plot(GT_trans(:,3),'k');hold on
   plot(scheme1(:,4),'r');hold on
  plot(scheme2(:,4),'g');hold on
 plot(scheme3(:,4),'b');hold on
    title( '相对于相机0时刻的位置-----------z'); 
      xlabel('黑-gt,红-scheme1,绿-scheme2,蓝-scheme3');
      ylabel('y/m');

      
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%输出位置误差
   disp(' 方案一 x方向误差')
      fprintf('   在x方向上的误差的均值为       %i\n', mean(abs(GT_trans(:,1)-scheme1(:,2))));
       fprintf('   在x方向上的误差最大的误差为   %i\n',abs(min(GT_trans(:,1)-scheme1(:,2))) );
       fprintf('   在x负方向上的误差最大的误差为 %i\n',abs(max(GT_trans(:,1)-scheme1(:,2))) );
       fprintf('   在x方向上的误差均方根为       %i\n', std(GT_trans(:,1)-scheme1(:,2)));
       disp('\n')
 disp(' 方案一   y方向误差')
      fprintf('   在y方向上的误差的均值为       %i\n', mean(abs(GT_trans(:,2)-scheme1(:,3))));
       fprintf('   在y方向上的误差最大的误差为   %i\n',abs(min(GT_trans(:,2)-scheme1(:,3))) );
       fprintf('   在y负方向上的误差最大的误差为 %i\n',abs(max(GT_trans(:,2)-scheme1(:,3))) );
       fprintf('   在y方向上的误差均方根为       %i\n', std(GT_trans(:,2)-scheme1(:,3)));
       disp('\n')
 disp('方案一   z方向误差')
      fprintf('   在z方向上的误差的均值为       %i\n', mean(abs(GT_trans(:,3)-scheme1(:,4))));
       fprintf('   在z方向上的误差最大的误差为   %i\n',abs(min(GT_trans(:,3)-scheme1(:,4))) );
       fprintf('   在z负方向上的误差最大的误差为 %i\n',abs(max(GT_trans(:,3)-scheme1(:,4))) );
       fprintf('   在z方向上的误差均方根为       %i\n', std(GT_trans(:,3)-scheme1(:,4)));
       disp('\n')
       
       
  disp('方案二  x方向误差')
      fprintf('   在x方向上的误差的均值为       %i\n', mean(abs(GT_trans(:,1)-scheme2(:,2))));
       fprintf('   在x方向上的误差最大的误差为   %i\n',abs(min(GT_trans(:,1)-scheme2(:,2))));
       fprintf('   在x负方向上的误差最大的误差为 %i\n', abs(max(GT_trans(:,1)-scheme2(:,2)))  );
       fprintf('   在x方向上的误差均方根为       %i\n', abs(std(GT_trans(:,1)-scheme2(:,2))));
       disp('\n')
  disp('方案二  y方向误差')
      fprintf('   在y方向上的误差的均值为       %i\n', mean(abs(GT_trans(:,2)-scheme2(:,3))));
       fprintf('   在y方向上的误差最大的误差为   %i\n',abs(min(GT_trans(:,2)-scheme2(:,3))));
       fprintf('   在y负方向上的误差最大的误差为 %i\n', abs(max(GT_trans(:,2)-scheme2(:,3)))  );
       fprintf('   在y方向上的误差均方根为       %i\n', abs(std(GT_trans(:,2)-scheme2(:,3))));
       disp('\n')
  disp('方案二  z方向误差')
      fprintf('   在z方向上的误差的均值为       %i\n', mean(abs(GT_trans(:,3)-scheme2(:,4))));
       fprintf('   在z方向上的误差最大的误差为   %i\n',abs(min(GT_trans(:,3)-scheme2(:,4))));
       fprintf('   在z负方向上的误差最大的误差为 %i\n', abs(max(GT_trans(:,3)-scheme2(:,4)))  );
       fprintf('   在z方向上的误差均方根为       %i\n', abs(std(GT_trans(:,3)-scheme2(:,4))));
       disp('\n')
       
    
   disp('方案三  x方向误差')
      fprintf('   在x方向上的误差的均值为       %i\n', mean(abs(GT_trans(:,1)-scheme3(:,2))));
       fprintf('   在x方向上的误差最大的误差为   %i\n',abs(min(GT_trans(:,1)-scheme3(:,2))));
       fprintf('   在x负方向上的误差最大的误差为 %i\n', abs( max(GT_trans(:,1)-scheme3(:,2)))  );
       fprintf('   在x方向上的误差均方根为       %i\n', std(GT_trans(:,1)-scheme3(:,2)));
       disp('\n')
  disp('方案三  y方向误差')
      fprintf('   在y方向上的误差的均值为       %i\n', mean(abs(GT_trans(:,2)-scheme3(:,3))));
       fprintf('   在y方向上的误差最大的误差为   %i\n',abs(min(GT_trans(:,2)-scheme3(:,3))));
       fprintf('   在y负方向上的误差最大的误差为 %i\n', abs( max(GT_trans(:,2)-scheme3(:,3)))  );
       fprintf('   在y方向上的误差均方根为       %i\n', std(GT_trans(:,2)-scheme3(:,3)));
       disp('\n')
  disp('方案三  z方向误差')
      fprintf('   在z方向上的误差的均值为       %i\n', mean(abs(GT_trans(:,3)-scheme3(:,4))));
       fprintf('   在z方向上的误差最大的误差为   %i\n',abs(min(GT_trans(:,3)-scheme3(:,4))));
       fprintf('   在z负方向上的误差最大的误差为 %i\n', abs( max(GT_trans(:,3)-scheme3(:,4)))  );
       fprintf('   在z方向上的误差均方根为       %i\n', std(GT_trans(:,3)-scheme3(:,4)));
       disp('\n')
 
 
  
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%相机姿态对比
    %%旋转角
     figure
     subplot(3,1,1);
  plot(GT_R(:,1),'k');grid;
  hold on
  plot(scheme1R(:,1),'r'); hold on
   
  plot(scheme2R(:,1),'g'); hold on
 
  plot(scheme3R(:,1),'b'); hold on
    title( '相对于相机0时刻的-----------偏转角');
       xlabel('帧');
      ylabel('y/度')
 subplot(3,1,2);
  plot(GT_R(:,2),'k');grid;
  hold on
   plot(scheme1R(:,2),'r');
  
   plot(scheme2R(:,2),'g'); hold on
  
  plot(scheme3R(:,2),'b'); hold on
    title( '相对于相机0时刻的-----------滚动角'); 
      xlabel('帧');
      ylabel('y/度')
 subplot(3,1,3);
  plot(GT_R(:,3),'k');grid;
  hold on
   plot(scheme1R(:,3),'r');
  
   plot(scheme2R(:,3),'g'); hold on
   
  plot(scheme3R(:,3),'b'); hold on
    title( '相对于相机0时刻的-----------俯仰角');
       xlabel('黑-gt,红-scheme1,绿-scheme2,蓝-scheme3');
      ylabel('y/度')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%输出R的误差    

       fprintf('  方案一在 旋转角上的误差       %i\n', abs(mean(GT_R(:,1)-scheme1R(:,1))));
       fprintf('  方案二在 旋转角上的误差       %i\n', abs(mean(GT_R(:,1)-scheme2R(:,1))));
       fprintf('  方案三在 旋转角上的误差       %i\n', abs(mean(GT_R(:,1)-scheme3R(:,1))));
       disp('\n')
        fprintf(' 方案一在 俯仰角上的误差       %i\n',abs( mean(GT_R(:,2)-scheme1R(:,2))));
       fprintf('  方案二在 俯仰角上的误差       %i\n', abs( mean(GT_R(:,2)-scheme2R(:,2))));
       fprintf('  方案三在 俯仰角上的误差       %i\n', abs(mean(GT_R(:,2)-scheme3R(:,2))));
       disp('\n')
        fprintf(' 方案一在 偏航角上的误差       %i\n',abs( mean(GT_R(:,3)-scheme1R(:,3))));
       fprintf('  方案二在 偏航角上的误差       %i\n',abs( mean(GT_R(:,3)-scheme2R(:,3))));
       fprintf('  方案三在 偏航角上的误差       %i\n', abs(mean(GT_R(:,3)-scheme3R(:,3))));
       disp('\n')
