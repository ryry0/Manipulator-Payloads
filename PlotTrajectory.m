% This script plots the trajectory very nice
% set of data.

t_f = 6;
num_links = 1;
time = 0:.001:t_f;
font_size = 16;
legend_font_size = 16;

set( gca                       , ...
    'FontName'   , 'Helvetica' );
if (num_links ==2)
    subplot (2,1,1);
    hModel = plot(time, data(:,2));
     set(hModel                        , ...
   'LineWidth'       , 1.5         );

    hXLabel = xlabel('Time (s)');
    hYLabel = ylabel('q_1 (rad)');
    
    set([hXLabel, hYLabel], ...
        'FontName'   , 'AvantGarde');
    
    set([hXLabel, hYLabel]  , ...
        'FontSize'   , font_size         );
    
    hold
    hModel = plot(time, data(:,3),'g');
    set(hModel                        , ...
        'LineWidth'       , 1.5         );
    
    hModel = plot(time, data(:,4),'r');
    set(hModel                        , ...
        'LineWidth'       , 1.5         );
    
    hLegend = legend ('q_1 position', 'q_1 velocity', 'q_1 acceleration');
    set([hLegend, gca]             , ...
        'FontSize'   , legend_font_size          );
    
    subplot (2,1,2);
    hModel = plot(time, data(:,5));
    set(hModel                        , ...
        'LineWidth'       , 1.5         );
    
    hXLabel = xlabel('Time (s)');
    hYLabel = ylabel('q_2 (rad)');
    
    set([hXLabel, hYLabel], ...
        'FontName'   , 'AvantGarde');
    
    set([hXLabel, hYLabel]  , ...
        'FontSize'   , font_size          );
    hold
    hModel = plot(time, data(:,6),'g');
    set(hModel                        , ...
        'LineWidth'       , 1.5         );
    hModel = plot(time, data(:,7),'r');
    set(hModel                        , ...
        'LineWidth'       , 1.5         );
    
    hLegend = legend ('q_2 position', 'q_2 velocity', 'q_2 acceleration');
    set([hLegend, gca]             , ...
        'FontSize'   , legend_font_size           );
    
elseif (num_links ==1)
    hModel = plot(time, data(:,5));
    set(hModel                        , ...
        'LineWidth'       , 1.5         );
    hXLabel = xlabel('Time (s)');
    hYLabel = ylabel('q_2 (rad)');
    
    set([hXLabel, hYLabel], ...
        'FontName'   , 'AvantGarde');
    
    set([hXLabel, hYLabel]  , ...
        'FontSize'   , font_size         );
    
    hold
    hModel = plot(time, data(:,6),'g');
    set(hModel                        , ...
        'LineWidth'       , 1.5         );
    
    hModel = plot(time, data(:,7),'r');
    set(hModel                        , ...
        'LineWidth'       , 1.5         );
    
    hLegend = legend ('q_2 position', 'q_2 velocity', 'q_2 acceleration');
    set([hLegend, gca]             , ...
        'FontSize'   , legend_font_size           );
    
end

% set(hFit                          , ...
%   'LineWidth'       , 2           );
% set(hE                            , ...
%   'LineWidth'       , 1           , ...
%   'Marker'          , 'o'         , ...
%   'MarkerSize'      , 6           , ...
%   'MarkerEdgeColor' , [.2 .2 .2]  , ...
%   'MarkerFaceColor' , [.7 .7 .7]  );
% set(hData                         , ...
%   'Marker'          , 'o'         , ...
%   'MarkerSize'      , 5           , ...
%   'MarkerEdgeColor' , 'none'      , ...
%   'MarkerFaceColor' , [.75 .75 1] );
% set(hCI(1)                        , ...
%   'LineWidth'       , 1.5         );
% set(hCI(2)                        , ...
%   'LineWidth'       , 1.5         );


% set( hTitle                    , ...
%     'FontSize'   , 12          , ...
%     'FontWeight' , 'bold'      );