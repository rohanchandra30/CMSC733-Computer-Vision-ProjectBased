function q = QuaternionNormalization(qs)
  q = bsxfun(@rdivide, qs, sqrt(sum(qs.^2)));
end