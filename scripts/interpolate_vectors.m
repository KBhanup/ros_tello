function [interp] = interpolate_vectors(ts,v)
    interp=[ts,zeros(size(ts,1),size(v,2)-1)];
    for i=1:size(ts,1)
       j=1;
       while(v(j,1)<ts(i))
           j=j+1;
       end
       if(v(j,1)==ts(i))
           interp(i,:)=v(j,:);
       else
           p=(ts(i)-v(j-1,1))/(v(j,1)-v(j-1,1));
           interp(i,:) = [ts(i),lerp(v(j-1,2:end),v(j,2:end),p)];
       end
    end
end

