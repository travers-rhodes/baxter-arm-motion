clear;
load data3.mat;

%create time lags
XLAG = lagmatrix(data,[0 1 2]);
X = XLAG(3:end,:);
[N,D] = size(X);

%feature generator function
Fnc = @(x)x2fx(x,'quadratic');

T=60;
max_iter = 100;
lambda = 10^-8;%lambda is the regularization strengt
P = 50; %number of trajectory snippets to use
tol=0.5;%tolerance of absolute forecast error
%validation set sizew needs to be larger than T
fit1 = DaD(X(1:300,:),X(300:400,:),Fnc,T,max_iter,lambda,P,tol);




xhat = zeros(T,D);
idx = 400;
xhat(1,:) = X(idx,:);
for t=2:T
  xhat(t,:) = Fnc(xhat(t-1,:))*fit1;
end

hold on; 
plot(X(idx:idx+T,1),'b'); 
plot(xhat(:,1),'r'); 

plot(X(idx:idx+T,2),'b'); 
plot(xhat(:,2),'r'); 

plot(X(idx:idx+T,3),'b'); 
plot(xhat(:,3),'r'); 
hold off;

