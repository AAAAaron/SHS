function floor_rec=floor_bypressure_matrix(t,pressure0,floor_status,height,acc)

nh=length(height);
height_matrix=zeros(nh);


for delt=1:nh-1
    for i=1:nh-delt      
        height_matrix(i,i+delt)=height_matrix(i,i+delt-1)+height(i+delt-1);
        height_matrix(i+delt,i)=height_matrix(i+delt-1,i)-height(i+delt-1);
    end
end

deltT=3;
thre=0.06;
n=length(pressure0);

pressure_before=median(pressure0(1:100));

pressure=pressure0*0;


j=1;
starti=ceil(t(1))+deltT;
buffer=[];
ti=[];
tm=[];
pr=[];
pm=[];
count=0;



deltheight_rec=[];
floor_rec=[];
k_rec=[];
acc_rec=[];
factor0=0.1
f_end=length(height);
pressure_before_rec=[];

num_avg=20;

flag_change=0;            %标识是否换层，上行为1，下行为-1，不换为0
scale=round(10/(t(11)-t(1)))       %为了接下来求拐点及之前斜率的，或者人工设定的固定气压取样时间间隔，换算成秒，然后算倒数

for i=1:n
    
    if i>num_avg
        pressure(i)=mean(pressure0(i-num_avg+1:i));
    else
        pressure(i)=pressure0(i);
    end

    if t(i)>starti
        buffer=[buffer,max(pressure(j:i))-min(pressure(j:i))];
        count=count+1;
        starti=starti+deltT;
        j=i;
        len_buffer=length(buffer);

        if (count==1 & buffer(end)<thre*factor0) | (count>1 & buffer(end-1)<thre*factor0)
            pressure_before=pressure(end);       %要不要低通    
        elseif buffer(end)>thre
            ti=[ti,t(i)];
            pr=[pr,pressure(i)];
        elseif count>1 & max(buffer(max(1,len_buffer-3):len_buffer-1))>thre
            tm=[tm,t(i)];
            pm=[pm,pressure(i)];
            pressure_end=pressure(i);  %确认稳态气压
            deltheight=-(pressure_end-pressure_before)/0.125;
            
            deltheight_rec=[deltheight_rec,deltheight];
            
            if (floor_status>1 & deltheight<-height(floor_status-1)*0.8)
                dist=abs(deltheight-height_matrix (floor_status,1:floor_status));
                [d1,ind]=min(dist);
%                 if ind~=floor_status
                    floor_status=ind;
                    pressure_before_rec=[pressure_before_rec,pressure_before];
                    pressure_before=pressure_end;  
%                 end
                 flag_change=1;            %-------向上换层
            elseif (floor_status<f_end & deltheight>height(floor_status)*0.8) 
                dist=abs(deltheight-height_matrix (floor_status,floor_status:nh));
                [d1,ind]=min(dist);
                    floor_status=floor_status+ind-1;
                    pressure_before_rec=[pressure_before_rec,pressure_before];
                    pressure_before=pressure_end;  
                 flag_change=-1;          %-------向下换层
            end
            
           %% 如果换层的话，继续求拐点，及之前的气压斜率 
            
            fous=3;
            if flag_change~=0
                kk=zeros(i,1);
                for j=max(scale*fous+1,i-scale*6):i-scale  %左侧不越界，往前推算6s    
                    kk(j)=flag_change*(pressure(j-scale*fous)+pressure(j+scale)-2*pressure(j));   
                end
                [~,ind_min]=min(kk);
                xtemp=1:scale*fous+1;
%                 k_temp=pressure(ind_min-scale*fous:ind_min)'/[xtemp',ones(scale*fous+1,1)]';
%                 k_change=k_temp(1)/0.125*scale*fous
                k_temp=LeastSquare(xtemp,pressure(ind_min-scale*fous:ind_min));   %调用最小二乘函数求k值
                k_change=k_temp/0.125*scale*fous
                acc_change=std(acc(ind_min-scale*fous:ind_min))
                
%                 class=recogn(k_change)              %只用“气压”来判断 电梯/楼梯/扶梯
                class2=recogn3(k_change,acc_change) %用“气压”和“加速度”来判断 电梯/楼梯/扶梯
                
                
                flag_change=0; %换层重置
                acc_rec=[acc_rec;[t(ind_min),acc_change]];
                k_rec=[k_rec;[t(ind_min),k_change]];
            end
            
            
        end 
        floor_rec=[floor_rec;[t(i),floor_status]];
        
    end
    
end




hco=figure;
subplot(3,1,1)
plot(t-t(1),pressure0,'m')
hold on
plot(t-t(1),pressure)



hold on
plot(ti-t(1),pr','r.')

hold on
plot(tm-t(1),pm,'mo')    
for i=1:length(tm)
text(tm(i)-t(1),pm(i),num2str(pm(i)))
end
set(gca,'xtick',1:2:t(end)-t(1))
xlim([0,t(end)-t(1)])
grid on


subplot(3,1,2)
plot(floor_rec(:,1)-t(1),floor_rec(:,2)*1,'g*')  
grid on
xlim([0,t(end)-t(1)])
set(gca,'xtick',1:2:t(end)-t(1))

subplot(3,1,3)
plot(k_rec(:,1)-t(1),k_rec(:,2),'r+') 
hold on
plot(acc_rec(:,1)-t(1),acc_rec(:,2),'g+') 
xlim([0,t(end)-t(1)])
grid on



end



