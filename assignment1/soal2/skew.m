function X = skew(vec)

X=zeros(length(vec));
X(1, 2) = -vec(3);
X(1, 3) = vec(2);
X(2, 1) = vec(3);
X(2, 3) = -vec(1);
X(3, 1) = -vec(2);
X(3, 2) = vec(1);