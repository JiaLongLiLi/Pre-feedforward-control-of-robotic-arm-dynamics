clc;clear;close all
%-------------------------------------
% author：知乎Winter 
% time：  2024-1-18
% puma560位置控制（动力学前馈+PID补偿）
% 智能机器人技术大作业
% 备注：用来让六个关节同时追踪sin曲线
%       dt时间步长会影响PID数值（过大），最好不要大于0.01，否则有时候导致accel函数出错
%       这里展示追踪sin

mdl_puma560

dt=0.01;
T = 20;
number=T/dt;

%----------------设置参数区域-----------------------------
% 跟踪轨迹产生
q_one = sin(linspace(0, T, number));
qd_one = cos(linspace(0, T, number));
qdd_one = -sin(linspace(0, T, number));
q=[q_one;q_one;q_one;q_one;q_one;q_one];
q_d=[qd_one;qd_one;qd_one;qd_one;qd_one;qd_one];
q_dd=[qdd_one;qdd_one;qdd_one;qdd_one;qdd_one;qdd_one];
% PID参数调节，有位置环和速度环
P_position=100;
I_position=0.1;
D_position=2;
P_velocity=1;
I_velocity=0.01;
D_velocity=0.01;

% 创建内存空间
q_actual=zeros(6,length(linspace(0, T, number)));
q_actual_d=zeros(6,length(linspace(0, T, number)));
tau_position_PID=zeros(6,length(linspace(0, T, number)));
tau_velocity_PID=zeros(6,length(linspace(0, T, number)));


integral_position_init=0;
integral_velocity_init=0;
for i=1:number-1

    if(mod(i,(1/dt))==0)
        percentageString = ['waitting: ',num2str(i/number*100), '%'];    
        % 打印进度结果
        disp(percentageString);
    end
    %前馈力矩
    tau_forward_temp= p560.rne(q(:,i)',q_d(:,i)',q_dd(:,i)');
      
    %PID力矩
    integral_position_init=integral_position_init+(q(:,i)-q_actual(:,i));
    tau_position_PID(:,i)=P_position.*(q(:,i)-q_actual(:,i))+...
                    I_position.* integral_position_init +...
                    D_position.*( q_d(:,i)-q_actual_d(:,i) );
    integral_velocity_init=integral_velocity_init+(tau_position_PID(:,i)-q_actual_d(:,i));
    if i==1
        tau_velocity_PID(:,i)=P_velocity.*(tau_position_PID(:,i) -q_actual_d(:,i))+...
                     I_velocity.*integral_velocity_init +...
                         D_velocity.*( (tau_position_PID(:,i)- zeros(6,1))/dt-...
                         (q_actual_d(:,i)-zeros(6,1))/dt);
    else
        tau_velocity_PID(:,i)=P_velocity.*(tau_position_PID(:,i) -q_actual_d(:,i))+...
                     I_velocity.*integral_velocity_init +...
                         D_velocity.*( (tau_position_PID(:,i)-tau_position_PID(:,i-1))/dt- ...
                         (q_actual_d(:,i)-q_actual_d(:,i-1))/dt);
    end
    %总力矩
    tau=tau_forward_temp+tau_velocity_PID(:,i)';
    %施加给机器人产生关节加速度
    qdd = p560.accel(q_actual(:,i)', q_actual_d(:,i)', tau);
    q_actual_d(:,i+1)=q_actual_d(:,i)+qdd.*dt;
    q_actual(:,i+1)=q_actual(:,i)+q_actual_d(:,i+1)*dt;
     
end
%画图数据
figure
plot(linspace(0, T, number),q_actual)

hold on
plot(linspace(0, T, number),q,'LineWidth',2)



