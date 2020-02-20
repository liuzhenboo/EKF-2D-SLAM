function f = landmarks(R1,R2,R3,a)

step = (a/180)*pi;
n1 = 360/a;
n2 = n1*2;
n3 = n1*3;
f = zeros(2,n3);
for i = 1:n1
    f(1,i) = R1*cos(i*step);
    f(2,i) = R1*sin(i*step);
end

for i = 1:n1
    f(1,i+n1) = R2*cos(i*step);
    f(2,i+n1) = R2*sin(i*step);
end

for i = 1:n1
    f(1,i+n2) = R3*cos(i*step);
    f(2,i+n2) = R3*sin(i*step);
end