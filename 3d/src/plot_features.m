% x is a 2 x n matrix
function plot_features( im, x, p )
    
    imshow(im)
    hold on
    for n = 1:size(x,2)
        plot(x(1,n),x(2,n), p);        
    end
    hold off
end

