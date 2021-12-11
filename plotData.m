function rCPp = plotData(q,p,trajectory)
    %% Helper Functions
    FK = Kinematics();
    %% Convert centerOfMass into foot space
    rCTt = zeros(3,p.N);
    rCPp = zeros(4,p.N);
    rSPp = zeros(3,p.N);
    for i = 1:p.N
        rCTt(:,i) = FK.CoM(q(:,i),p);
        rSPp(:,i) = FK.xe(q(:,i),p);
        Htp = FK.Htp(q(:,i),p);
        rCPp(:,i) = Htp\[rCTt(:,i);1];
    end
    %% Plot CoM position
    figure
    n = p.N;
    subplot(2,1,1);
    hold on;
    plot(ones(1,n)*-0.115,'--','LineWidth',5);
    hold on;
    plot(ones(1,n)*0.115,'--','LineWidth',5);
    hold on;
    plot(rCPp(3,:),'LineWidth',5);
    xlim([1 length(rCPp)])
    legend('CoM upper limit [m]','CoM lower limit [m]','CoM position [m] (support foot space)');
    title('X CoM constraint')
    xlabel('Sample ΔT')
    ylabel('CoM X position [m]')
    ylim([-0.2 0.2]);
    subplot(2,1,2);
    hold on;
    plot(ones(1,n)*0.13/2,'--','LineWidth',5);
    hold on;
    plot(ones(1,n)*-0.13/2,'--','LineWidth',5);
    hold on;
    plot(rCPp(2,:),'LineWidth',5);
    legend('CoM upper limit [m]','CoM lower limit [m]','CoM position [m] (support foot space)');
    title('Y CoM constraint')
    xlabel('Sample ΔT')
    ylabel('CoM Y position [m]')
    ylim([-0.2 0.2]);
    xlim([1 length(rCPp)])
    %% Plot Trajectory vs End effector position
    figure
    subplot(3,1,1);
    plot(trajectory(1,:),'LineWidth',5);
    hold;
%     plot(rSPp(1,:),'--','LineWidth',5);
    xlim([1 length(rCPp)])
%     legend('Swing Foot X Achieved  [m] (support foot space)','Swing Foot X Desired [m] (support foot space)');
    title('Swing foot trajectory X [m]')
    xlabel('Sample ΔT')
    ylabel('Swing Foot X position [m]')
    subplot(3,1,2);
    plot(trajectory(2,:),'LineWidth',5);
    ylim([-0.2 0.2]);
    xlim([1 length(rCPp)])
    hold;
%     plot(rSPp(2,:),'--','LineWidth',5);
%     legend('Swing Foot Y Achieved [m] (support foot space)','Swing Foot Y Desired [m] (support foot space)');
    title('Swing foot trajectory Y [m]')
    xlabel('Sample ΔT')
    ylabel('Swing Foot Y position [m]')
    subplot(3,1,3);
    plot(trajectory(3,:),'LineWidth',5);
    hold;
%     plot(rSPp(3,:),'--','LineWidth',5);
    xlim([1 length(rCPp)])
%     legend('Swing Foot Z Achieved [m] (support foot space)','Swing Foot Z Desired [m] (support foot space)');
    title('Swing foot trajectory Z [m]')
    xlabel('Sample ΔT')
    ylabel('Swing Foot Z position [m]')
end

