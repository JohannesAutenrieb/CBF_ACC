
function plotResults_ZCBF(ts, xs, us, slacks, hs, Vs, params, sim)
    fig_sz = [10 15]; 
    plot_pos = [0 0 10 15];
    yellow = [0.998, 0.875, 0.529];
    blue = [0.106, 0.588, 0.953];
    navy = [0.063, 0.075, 0.227];
    magenta = [0.937, 0.004, 0.584];
    orange = [0.965, 0.529, 0.255];
    grey = 0.01 *[19.6, 18.8, 19.2];
    
    figure('Color', 'w','units','normalized','outerposition',[0 0 1 1]);
%     set(gcf, 'Color', 'w');
    subplot(3,2,1);
    p = plot(ts, xs(:, 2));
    p.Color = blue;
    p.LineWidth = 1.5;
    hold on;
    plot(ts, params.vd*ones(size(ts, 1), 1), 'k--');
    xlim([min(ts) max(ts)]);
    ylabel("v (m/s)");
    title("State - Velocity");
    set(gca,'FontSize',14);
    grid on;    
    

    subplot(3,2,2);
    p = plot(ts, xs(:, 3));
    xlim([min(ts) max(ts)]);
    p.Color = magenta;
    p.LineWidth = 1.5;
    ylabel("z (m)");
    title("State - Distance to lead vehicle");
    set(gca, 'FontSize', 14);
    grid on;    
    
    subplot(3,2,3);
    p = plot(ts(1:end-1), us); hold on;
    p.Color = orange;
    p.LineWidth = 1.5;
    plot(ts(1:end-1), params.u_max*ones(size(ts, 1)-1, 1), 'r--');
    plot(ts(1:end-1), params.u_min*ones(size(ts, 1)-1, 1), 'r--');
    ylim([1.5*params.u_min 1.5*params.u_max]);
    ylabel("u(N)");
    title("Control Input - Wheel Force");    
    set(gca, 'FontSize', 14);
    grid on;    

    subplot(3,2,4);
    p = plot(ts(1:end-1), slacks); hold on;
    p.Color = magenta;
    p.LineWidth = 1.5;
    ylabel("slack");
    title("Slack variable");        
    set(gca, 'FontSize', 14);
    grid on;    

    
    subplot(3,2,5);
    p = plot(ts(1:end-1), hs);
    p.Color = navy;
    p.LineWidth = 1.5;
    ylabel("CBF (h(x))");
    xlabel("t(s)");
    title("CBF");    
    set(gca, 'FontSize', 14);
        grid on;    

    subplot(3,2,6);
    set(gca, 'FontSize', 14);
    p = plot(ts(1:end-1), Vs);
    p.Color = navy;
    p.LineWidth = 1.5;
    xlabel("t(s)");
    ylabel("CLF (V(x))");
    title("CLF");    
    set(gca, 'FontSize', 14);
    grid on;    

    set(gcf, 'PaperSize', fig_sz);
    set(gcf, 'PaperPosition', plot_pos);
    
    set(gcf,'Units','inches');
    screenposition = get(gcf,'Position');
    set(gcf,...
    'PaperPosition',[0 0 screenposition(3:4)],...
    'PaperSize',[screenposition(3:4)]);
    saveas(gcf,[sim.resultsPath '.pdf'])
    export_fig(sim.resultsPath, '-png')

end
