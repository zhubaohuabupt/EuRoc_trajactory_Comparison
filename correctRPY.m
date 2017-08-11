%作者：朱保华
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%函属功能：处理角度临界突变%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [dataafter]=correctRPY(databefor)
[row col]=size(databefor);
lastr=0;
lastp=0;
lasty=0;
m=0;
for i=1:1:row
    r=databefor(i,1);
    p=databefor(i,2);
    y=databefor(i,3);
    
     if (r-lastr)<-200
            
      dataafter(i,1)=r+360;
       if (r-lastr)<-400
           dataafter(i,1)=r+720;
       end

      else
       dataafter(i,1)=r;
      end
    
     dataafter(i,2)=p;

      if (y-lasty)<-200
            
         dataafter(i,3)=y+360   ;   
         m=dataafter(i,3);
      else
       dataafter(i,3)=y;
      end
      
      
      if(y-lasty)>200
          dataafter(i,3)=y-360;        
      end
  
   lasty=dataafter(i,3);
    
   lastr=dataafter(i,1);
end
end