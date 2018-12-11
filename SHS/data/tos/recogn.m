function label=recogn(k)
if abs(k)<1
    '·öÌÝ'
    label=4;
elseif abs(k)>2.2
    'Ö±ÌÝ'
    label=3;
else
    'Â¥ÌÝ'
    label=5;    
end
