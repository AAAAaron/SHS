function label=recogn3(k,a)
if abs(a)>1 & (abs(k)>0.85 & abs(k)<2.9)
    '¥��'
    label=5;   
elseif k<-0.3 & abs(a)>0.5 
    '¥��'
    label=5;       
elseif abs(k)>2.4
    'ֱ��'
    label=3;
else
    '����'
    label=4;
end
