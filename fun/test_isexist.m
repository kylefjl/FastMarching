A=[2 1 -5 1;
   1 -3 0 -6;
   0 2 -1 2;
   1 4 -7 6;];
b=[8;9;-5;0];
y=isexist(A,b)
function y=isexist(A,b)
[m,n]=size(A);
[mb,nb]=size(b);
if m~=mb
    error;
    return;
end
r=rank(A);
s=rank([A,b]);
if r==s &&r==n
    y=1;
elseif r==s&&r<n
    y=Inf
else
    y=0
end
end
