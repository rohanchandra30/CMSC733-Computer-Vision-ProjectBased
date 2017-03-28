function cp = GetCameraPosition(CP, K)
  RT = K\CP; 
  R = RT(:,1:3);
  T = RT(:,4);
  cp = (R\T).*-1;
end