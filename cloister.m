function f = cloister(xmin,xmax,ymin,ymax,n)


if nargin < 5
    n = 9;
end

x0 = (xmin+xmax)/2;
y0 = (ymin+ymax)/2;

hsize = xmax-xmin;
vsize = ymax-ymin;
tsize = diag([hsize vsize]);

midle  = (n-5)/2;
int    = (n-3)/2;
ext    = (n-1)/2;
%[-2:2]
cmidle = [-midle:midle];
cint   = [cmidle int];
cext   = [-int cint];
zint   = zeros(1,n-3);
zext   = zeros(1,n-2);

northint = [cint;zint+int];
northext = [cext;zext+ext];

southint = [-cint;zint-int];
southext = [-cext;zext-ext];

eastint  = [zint+int;-cint];
eastext  = [zext+ext;-cext];

westint  = [zint-int;cint];
westext  = [zext-ext;cext];


f = [northint northext southint southext eastint eastext westint westext];

f = tsize*f/(n-1);

f(1,:) = f(1,:) + x0;
f(2,:) = f(2,:) + y0;
