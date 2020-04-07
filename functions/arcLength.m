function L = arcLength(az,el,rad)
L = rad*distance(az(1:end-1),el(1:end-1),az(2:end),el(2:end));
L = trapz(L);
end