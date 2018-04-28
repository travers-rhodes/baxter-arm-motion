function beta_best = DaD(X,Xval,fnc,T,N,lambda,P,tol)
% X is the time series of state space observation, each row is an obs.
% fnc is the featurization function
% Xval is a validation set
% T is the prediction horizon
% N is the max number of iterations
% lambda is the regularization strength
% P is the number of trajectory snippets to use
% tol is the distance (box) of tolerance, if the prediction differs by more
%          than tol in any direction, the forecast is terminated

function [X_,Y_,t_] = forecast(X,beta,fnc,T)
  idx_ = datasample(1:size(X,1)-T-2,P)';
  n_ = length(idx_);
  X_ = zeros(n_*T,size(X,2));
  Y_ = zeros(n_*T,size(X,2));
  X_(1:n_,:) = fnc(X(idx_,:))*beta;
  Y_(1:n_,:) = X(idx_+3,:);
  for t_=2:T
    X_(1+(t_-1)*n_:t_*n_,:) = fnc(X_(1+(t_-2)*n_:(t_-1)*n_,:))*beta;
    Y_(1+(t_-1)*n_:t_*n_,:) = X(idx_+t_+2,:);
    if sum(sum(~isfinite(X_)))>0, break; end;
  end
  X_ = X_(1:t_*n_,:);
  Y_ = Y_(1:t_*n_,:);
  g = max(abs(X_-Y_),[],2)<tol;
  X_ = X_(g,:);
  Y_ = Y_(g,:);
end

function [max_err,t_] = validate(X,beta,fnc,T)
  idx_ = datasample(1:size(X,1)-T-2,P)'; 
  max_err = 0;
  t_ = 1;
  Xhat = fnc(X(idx_,:))*beta;
  for t_=2:T
    Xhat = fnc(Xhat)*beta;
    if sum(sum(~isfinite(Xhat)))>0, t_=t_-1; break; end;
    max_err = max_err+sum(sum((Xhat-X(idx_+t_+1,:)).^2));
    %if err>max_err, max_err=err; end;
  end
end

beta_best = 0; h_best = 0; err_best = inf;

n = size(X,1)-2;
m = size(fnc(X(1:2,:)),2);
I = lambda*spdiags(ones(m,1),0,m,m);
X_ = fnc(X(1:end-2,:));
XX_ = X_'*X_/n;
XY_ = X_'*X(3:end,:)/n;
beta = (XX_+I)\XY_;
for i=2:N
  [err,h_val] = validate(Xval,beta,fnc,T);
  [X_,Y_,h_train] = forecast(X,beta,fnc,T);
  if h_val>h_best || (h_val==h_best && err<err_best)
     h_best = h_val;
     err_best = err;
     beta_best = beta;
  end
  %[i,h_train,h_val,err,err_best]
  X_ = fnc(X_);
  XX_ = XX_*(n/(n+size(X_,1))) + X_'*X_/(n+size(X_,1));
  XY_ = XY_*(n/(n+size(X_,1))) + X_'*Y_/(n+size(X_,1));
  n = n + size(X_,1);
  beta = (XX_+I)\XY_;
end

end
