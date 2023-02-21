function [aligned] = align_times(base,unaligned)
%Assume the unaligned is sorted by time
    aligned = [base(:,1),zeros(size(base,1),size(unaligned,2)-1)];
    a=1;
    b=1;
    for a=1:size(base,1)
        closest = unaligned(b,1);
        %while the current one is closer to the goal time than the last,
        %increment b
        while(abs(unaligned(b+1,1)-base(a,1))<abs(closest-base(a,1)) && b<size(unaligned,1))
            b=b+1;
            closest = unaligned(b,1);
        end
        %the previous loop will go one too far, so decrement
        aligned(a,2:end) = unaligned(b,2:end);
    end
end

