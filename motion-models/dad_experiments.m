clear;
for dfname = 1:6
    fname = sprintf('data%d-30sps.mat',1);
    load(fname);

    %create time lags
    XLAG = lagmatrix(data,[0 1 2]);
    X = XLAG(3:end,:);
    [N,D] = size(X);

    %feature generator function
    Fnc = @(x)x2fx(x,'quadratic');


    tstart = 30;
    tend = 300;
    indexes = 30:300;
    horizon = 60;
    results = zeros(length(indexes), horizon);

    max_iter = 600;
    lambda = 10^-8;%lambda is the regularization strengt

    tol=0.5;%tolerance of absolute forecast error


    %validation set sizew needs to be larger than T
    count = 1;
    parfor numpoints = indexes
        train = ceil(numpoints/3.0)*2;
        val = numpoints-train;

        P = floor(val*0.7); %number of trajectory snippets to use
        T = floor(val*0.6);
        fit3 = DaD(X(1:train,:),X(train:numpoints,:),Fnc,T,max_iter,lambda,P,tol);
        %fit3 = DaD(X(1:train,:),X(train:numpoints,:),Fnc,T,max_iter,lambda,P,tol);

        xhat = zeros(60,D);
        idx = numpoints;
        xhat(1,:) = X(idx,:);
        for t=2:60
          xhat(t,:) = Fnc(xhat(t-1,:))*fit3;
        end

        error = sqrt(sum((X(idx:idx+60-1,1:3)-xhat(:,1:3)).^2,2));
        results(numpoints-29,:)=error;
        count= count+1;
    end
    save(sprintf('results%d.mat',dfname),'results');
end
% 
% hold on; 
% plot(X(idx:end,1),'b'); 
% plot(xhat(:,1),'r'); 
% 
% plot(X(idx:end,2),'b'); 
% plot(xhat(:,2),'r'); 
% 
% plot(X(idx:end,3),'b'); 
% plot(xhat(:,3),'r'); 
% hold off;