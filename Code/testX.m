function [t1, t2] = testX(X, C1, R1, R2, C2, K)
  P1 = K * [R1 (R1\C1).*-1];
  P2 = K * [R1 (R2\C2).*-1]; 
  X_homo = [X ones(length(X),1)];
  for i =1:length(X)
    t1(i,:) = P1*X_homo(i,:)';
    t2(i,:) = P2*X_homo(i,:)';
  end
    t1 = t1(:,1:2)./t1(:,3);
    t2 = t2(:,1:2)./t2(:,3);
end