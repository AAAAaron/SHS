function label=recogn(k)
if abs(k)<1
    '����'
    label=4;
elseif abs(k)>2.2
    'ֱ��'
    label=3;
else
    '¥��'
    label=5;    
end
