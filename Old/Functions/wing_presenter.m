function h_2 = wing_presenter(Resolution,r,wing_effects)

s_1 = Resolution;
s_2 = 2.*Resolution;
s_3 = 3.*Resolution;
s_4 = 4.*Resolution;

h_2 = figure(5);

subplot(4,1,1);
h_2(1) = plot(r,wing_effects(1,1:s_1));
title('dL');
xlabel('r');
ylabel('(N)');

subplot(4,1,2);
h_2(2) = plot(r,wing_effects(1,s_1+1:s_2));
title('dD');
xlabel('r');
ylabel('(N)');

subplot(4,1,3);
h_2(3) = plot(r,wing_effects(1,s_2+1:s_3));
title('AoA');
xlabel('r');
ylabel('(rad)');

subplot(4,1,4);
h_2(4) = plot(r,wing_effects(1,s_3+1:s_4));
title('v_i');
xlabel('r');
ylabel('(m/s)');

slide = uicontrol('Style','slider','Position',[400 10 120 20],...
                  'Min',1,'Max',size(wing_effects,2),'Value',1,...
                  'Callback',@sliderCallback);
    
function sliderCallback(hObject,evt)
    time = round(get(hObject, 'Value'));
    
    subplot(4,1,1);
    h_2(1) = plot(r,wing_effects(time,1:s_1));
    title('dL');
    xlabel('r');
    ylabel('(N)');

    subplot(4,1,2);
    h_2(2) = plot(r,wing_effects(time,s_1+1:s_2));
    title('dD');
    xlabel('r');
    ylabel('(N)');

    subplot(4,1,3);
    h_2(3) = plot(r,wing_effects(time,s_2+1:s_3));
    title('AoA');
    xlabel('r');
    ylabel('(rad)');

    subplot(4,1,4);
    h_2(4) = plot(r,wing_effects(time,s_3+1:s_4));
    title('v_i');
    xlabel('r');
    ylabel('(m/s)');
    legend(num2str(25.*time./size(wing_effects,2)));    % Assumes data over 25s
end
    
end
