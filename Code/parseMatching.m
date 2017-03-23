function [matches, fpoints] = parseMatching(Nimages, matchdata, i) 
  
  nFeatures = matchdata(1,2);
  fpoints = cell(nFeatures, Nimages);
    
  for row = 2:nFeatures+1
   fpoints{row,i} = [matchdata(row,5) matchdata(row,6)];
   n_f_m = matchdata(row,1);
   ref = 7;
   for j = 1:n_f_m-1
          im = matchdata(row,ref);
          fpoints{row,im} = [matchdata(row,ref+1) matchdata(row,ref+2)];
          ref = ref +3;
   end    
  end
  p1 = fpoints(2:end,i);
  for j = i+1:Nimages
        p2 = fpoints(2:end,j);
        ps1 = p1(~cellfun(@isempty, p2));
        ps2 = p2(~cellfun(@isempty, p2));
        matches{j,1} = [ps1, ps2];
   end        
end  