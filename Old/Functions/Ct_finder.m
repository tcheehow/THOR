function a = Ct_finder(prop_fs_list,Ct_list,prop_fs)

% edit double interpolator to this

checker = 1;

for i = 1 : (length(prop_fs_list)-1)
    if prop_fs >= prop_fs_list(i) && prop_fs < prop_fs_list(i+1)
        m = (Ct_list(i+1)-Ct_list(i))/(prop_fs_list(i+1)-prop_fs_list(i));
        a = m.*(prop_fs-prop_fs_list(i))+Ct_list(i);
        checker = 0;
    end
end

if checker == 1
    a = 0;
%    debug = 'Ct_finder has exceeded database boundaries. Assuming Ct to be 0.'
end

