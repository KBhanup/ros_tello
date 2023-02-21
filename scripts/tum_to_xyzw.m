function xyzw = tum_to_xyzw(tum)
    N=size(tum,1);
    xyzw=zeros(N,5);
    for i=1:N
        [heading,elevation,roll] = quat_to_euler(tum(i,5:8));
        if(heading>pi)
            heading= heading-2*pi;
        end
        xyzw(i,:) = [tum(i,1:4),heading];
    end
end

