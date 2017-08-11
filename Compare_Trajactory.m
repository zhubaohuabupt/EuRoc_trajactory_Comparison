%���ߣ��챣��
%�����������


  %Scheme1��3�� ��ȡorb�����λ�ˣ����ݲ��Եķ���������ͬ������ѡ��ע�ͻ�����µķ�����
 scheme1 =importdata('D:\matlab_program\compare_trajactory\202\CameraTrajectory2021.txt');%   ����1
 scheme2 =importdata('D:\matlab_program\compare_trajactory\202\CameraTrajectory202_getfts_chang.txt');%����2
 scheme3 =importdata('D:\matlab_program\compare_trajactory\202\CameraTrajectory202_getfts_chang.txt');%����3

 %��ȡ������ݵĴ�С���Ա������ͬ��С��groudtruth��С�����ݡ�
 [rows cols]=size(scheme1);
 
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%��groundtruth�Ĵ���%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %groundtruth��EvRocͼ���ṩ�����ݣ���imu��������������λ�ˡ�
 groundtruth=csvread('F:\slam_Trajectory\Vgroundtruth\V202\data.csv',7,0);%��       7����������
 %�����ṩ��groundtruth����ȡ�������������0ʱ�̵�λ���Լ�֡��仯�������get_cam_gt����ע�͡�
   [GT_trans,GT_Rotation,GT_deta_trans,GT_deta_R,GT_deta_trans_inv,GT_deta_R_inv]=get_cam_gt(groundtruth,rows);
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%��slam������ݵĴ���%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
  %��orb�����λ�������л�ȡ��ת���󣬲�ȥ���ٽ�ͻ�䡣
   scheme1RR=getorbslamR(scheme1);
   scheme2RR=getorbslamR(scheme2);
   scheme3RR=getorbslamR(scheme3);
   scheme1R=correctRPY(scheme1RR);
   scheme2R=correctRPY(scheme2RR);
   scheme3R=correctRPY(scheme3RR);
    GT_R=correctRPY(GT_Rotation);
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%��ʼ�ԱȾ��ȣ�%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%             
   figure 
   subplot(3,1,1)
  plot(GT_trans(:,1),'k');hold on
  plot(scheme1(:,2),'r');hold on
  plot(scheme2(:,2),'g');hold on
 plot(scheme3(:,2),'b');hold on
   title( '��������0ʱ�̵�λ��-----------x'); 
      xlabel('֡');
      ylabel('y/m')
subplot(3,1,2)
 plot(GT_trans(:,2),'k');hold on 
  plot(scheme1(:,3),'r');hold on
 plot(scheme2(:,3),'g');hold on
 plot(scheme3(:,3),'b');hold on
    title( '��������0ʱ�̵�λ��-----------y'); 
      xlabel('֡');
      ylabel('y/m');
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%���λ�öԱ�
 subplot(3,1,3)
   plot(GT_trans(:,3),'k');hold on
   plot(scheme1(:,4),'r');hold on
  plot(scheme2(:,4),'g');hold on
 plot(scheme3(:,4),'b');hold on
    title( '��������0ʱ�̵�λ��-----------z'); 
      xlabel('��-gt,��-scheme1,��-scheme2,��-scheme3');
      ylabel('y/m');

      
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%���λ�����
   disp(' ����һ x�������')
      fprintf('   ��x�����ϵ����ľ�ֵΪ       %i\n', mean(abs(GT_trans(:,1)-scheme1(:,2))));
       fprintf('   ��x�����ϵ�����������Ϊ   %i\n',abs(min(GT_trans(:,1)-scheme1(:,2))) );
       fprintf('   ��x�������ϵ�����������Ϊ %i\n',abs(max(GT_trans(:,1)-scheme1(:,2))) );
       fprintf('   ��x�����ϵ���������Ϊ       %i\n', std(GT_trans(:,1)-scheme1(:,2)));
       disp('\n')
 disp(' ����һ   y�������')
      fprintf('   ��y�����ϵ����ľ�ֵΪ       %i\n', mean(abs(GT_trans(:,2)-scheme1(:,3))));
       fprintf('   ��y�����ϵ�����������Ϊ   %i\n',abs(min(GT_trans(:,2)-scheme1(:,3))) );
       fprintf('   ��y�������ϵ�����������Ϊ %i\n',abs(max(GT_trans(:,2)-scheme1(:,3))) );
       fprintf('   ��y�����ϵ���������Ϊ       %i\n', std(GT_trans(:,2)-scheme1(:,3)));
       disp('\n')
 disp('����һ   z�������')
      fprintf('   ��z�����ϵ����ľ�ֵΪ       %i\n', mean(abs(GT_trans(:,3)-scheme1(:,4))));
       fprintf('   ��z�����ϵ�����������Ϊ   %i\n',abs(min(GT_trans(:,3)-scheme1(:,4))) );
       fprintf('   ��z�������ϵ�����������Ϊ %i\n',abs(max(GT_trans(:,3)-scheme1(:,4))) );
       fprintf('   ��z�����ϵ���������Ϊ       %i\n', std(GT_trans(:,3)-scheme1(:,4)));
       disp('\n')
       
       
  disp('������  x�������')
      fprintf('   ��x�����ϵ����ľ�ֵΪ       %i\n', mean(abs(GT_trans(:,1)-scheme2(:,2))));
       fprintf('   ��x�����ϵ�����������Ϊ   %i\n',abs(min(GT_trans(:,1)-scheme2(:,2))));
       fprintf('   ��x�������ϵ�����������Ϊ %i\n', abs(max(GT_trans(:,1)-scheme2(:,2)))  );
       fprintf('   ��x�����ϵ���������Ϊ       %i\n', abs(std(GT_trans(:,1)-scheme2(:,2))));
       disp('\n')
  disp('������  y�������')
      fprintf('   ��y�����ϵ����ľ�ֵΪ       %i\n', mean(abs(GT_trans(:,2)-scheme2(:,3))));
       fprintf('   ��y�����ϵ�����������Ϊ   %i\n',abs(min(GT_trans(:,2)-scheme2(:,3))));
       fprintf('   ��y�������ϵ�����������Ϊ %i\n', abs(max(GT_trans(:,2)-scheme2(:,3)))  );
       fprintf('   ��y�����ϵ���������Ϊ       %i\n', abs(std(GT_trans(:,2)-scheme2(:,3))));
       disp('\n')
  disp('������  z�������')
      fprintf('   ��z�����ϵ����ľ�ֵΪ       %i\n', mean(abs(GT_trans(:,3)-scheme2(:,4))));
       fprintf('   ��z�����ϵ�����������Ϊ   %i\n',abs(min(GT_trans(:,3)-scheme2(:,4))));
       fprintf('   ��z�������ϵ�����������Ϊ %i\n', abs(max(GT_trans(:,3)-scheme2(:,4)))  );
       fprintf('   ��z�����ϵ���������Ϊ       %i\n', abs(std(GT_trans(:,3)-scheme2(:,4))));
       disp('\n')
       
    
   disp('������  x�������')
      fprintf('   ��x�����ϵ����ľ�ֵΪ       %i\n', mean(abs(GT_trans(:,1)-scheme3(:,2))));
       fprintf('   ��x�����ϵ�����������Ϊ   %i\n',abs(min(GT_trans(:,1)-scheme3(:,2))));
       fprintf('   ��x�������ϵ�����������Ϊ %i\n', abs( max(GT_trans(:,1)-scheme3(:,2)))  );
       fprintf('   ��x�����ϵ���������Ϊ       %i\n', std(GT_trans(:,1)-scheme3(:,2)));
       disp('\n')
  disp('������  y�������')
      fprintf('   ��y�����ϵ����ľ�ֵΪ       %i\n', mean(abs(GT_trans(:,2)-scheme3(:,3))));
       fprintf('   ��y�����ϵ�����������Ϊ   %i\n',abs(min(GT_trans(:,2)-scheme3(:,3))));
       fprintf('   ��y�������ϵ�����������Ϊ %i\n', abs( max(GT_trans(:,2)-scheme3(:,3)))  );
       fprintf('   ��y�����ϵ���������Ϊ       %i\n', std(GT_trans(:,2)-scheme3(:,3)));
       disp('\n')
  disp('������  z�������')
      fprintf('   ��z�����ϵ����ľ�ֵΪ       %i\n', mean(abs(GT_trans(:,3)-scheme3(:,4))));
       fprintf('   ��z�����ϵ�����������Ϊ   %i\n',abs(min(GT_trans(:,3)-scheme3(:,4))));
       fprintf('   ��z�������ϵ�����������Ϊ %i\n', abs( max(GT_trans(:,3)-scheme3(:,4)))  );
       fprintf('   ��z�����ϵ���������Ϊ       %i\n', std(GT_trans(:,3)-scheme3(:,4)));
       disp('\n')
 
 
  
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%�����̬�Ա�
    %%��ת��
     figure
     subplot(3,1,1);
  plot(GT_R(:,1),'k');grid;
  hold on
  plot(scheme1R(:,1),'r'); hold on
   
  plot(scheme2R(:,1),'g'); hold on
 
  plot(scheme3R(:,1),'b'); hold on
    title( '��������0ʱ�̵�-----------ƫת��');
       xlabel('֡');
      ylabel('y/��')
 subplot(3,1,2);
  plot(GT_R(:,2),'k');grid;
  hold on
   plot(scheme1R(:,2),'r');
  
   plot(scheme2R(:,2),'g'); hold on
  
  plot(scheme3R(:,2),'b'); hold on
    title( '��������0ʱ�̵�-----------������'); 
      xlabel('֡');
      ylabel('y/��')
 subplot(3,1,3);
  plot(GT_R(:,3),'k');grid;
  hold on
   plot(scheme1R(:,3),'r');
  
   plot(scheme2R(:,3),'g'); hold on
   
  plot(scheme3R(:,3),'b'); hold on
    title( '��������0ʱ�̵�-----------������');
       xlabel('��-gt,��-scheme1,��-scheme2,��-scheme3');
      ylabel('y/��')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%���R�����    

       fprintf('  ����һ�� ��ת���ϵ����       %i\n', abs(mean(GT_R(:,1)-scheme1R(:,1))));
       fprintf('  �������� ��ת���ϵ����       %i\n', abs(mean(GT_R(:,1)-scheme2R(:,1))));
       fprintf('  �������� ��ת���ϵ����       %i\n', abs(mean(GT_R(:,1)-scheme3R(:,1))));
       disp('\n')
        fprintf(' ����һ�� �������ϵ����       %i\n',abs( mean(GT_R(:,2)-scheme1R(:,2))));
       fprintf('  �������� �������ϵ����       %i\n', abs( mean(GT_R(:,2)-scheme2R(:,2))));
       fprintf('  �������� �������ϵ����       %i\n', abs(mean(GT_R(:,2)-scheme3R(:,2))));
       disp('\n')
        fprintf(' ����һ�� ƫ�����ϵ����       %i\n',abs( mean(GT_R(:,3)-scheme1R(:,3))));
       fprintf('  �������� ƫ�����ϵ����       %i\n',abs( mean(GT_R(:,3)-scheme2R(:,3))));
       fprintf('  �������� ƫ�����ϵ����       %i\n', abs(mean(GT_R(:,3)-scheme3R(:,3))));
       disp('\n')
