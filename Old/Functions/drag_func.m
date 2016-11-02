function a = drag_func(AoA,airfoil_data)

count = size(airfoil_data,1);

a = 0;

for k = 1 : (count-1)
    if AoA >= airfoil_data(k,1) && AoA < airfoil_data(k+1,1)
        a = 1./airfoil_data(k,4);
        debug = 'drag calc success';
    elseif AoA < airfoil_data(1,1)
        a = -1./2 ;                                  % assumed worst case drag. Temporary
        debug = 'drag calc problem. using temp solution (-ve)';
    elseif AoA > airfoil_data(end,1)
        a = 1./2 ;
        debug = 'drag calc problem. using temp solution (+ve)';   
    end
end