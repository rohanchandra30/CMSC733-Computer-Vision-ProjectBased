function [Cset, Rset] = Do_Correction(C1, C2, C3, C4, R1, R2, R3, R4)


if (det(R1)==-1)
   Cset{1} = -1.*C1; Rset{1} = -1.*R1; 
else
    Cset{1} = 1.*C1; Rset{1} = 1.*R1; 
   
end

if (det(R2)==-1)
   Cset{2} = -1.*C2; Rset{2} = -1.*R2; 
   else
    Cset{2} = 1.*C2; Rset{2} = 1.*R2; 
   
end

if (det(R3)==-1)
   Cset{3} = -1.*C3; Rset{3} = -1.*R3; 
   else
    Cset{3} = 1.*C3; Rset{3} = 1.*R3; 
   
end

if (det(R4)==-1)
   Cset{4} = -1.*C4; Rset{4} = -1.*R4; 
   else
    Cset{4} = 1.*C4; Rset{4} = 1.*R4; 
   
end

end