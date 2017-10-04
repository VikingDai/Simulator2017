
i =1;
X = (test(:,1)+101)';
Y = (test(:,2)+101)';
for x = X
    for y =  Y
        mat(x,y) = test(i,3);
        i=i+1;
    end
end

surf(mat)