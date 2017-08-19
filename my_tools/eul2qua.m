function q = eul2qua(e)
    x1 = cos(e(1)/2)*cos(e(2)/2)*cos(e(3)/2) + sin(e(1)/2)*sin(e(2)/2)*sin(e(3)/2);
    x2 = sin(e(1)/2)*cos(e(2)/2)*cos(e(3)/2) - cos(e(1)/2)*sin(e(2)/2)*sin(e(3)/2);
    x3 = cos(e(1)/2)*sin(e(2)/2)*cos(e(3)/2) + sin(e(1)/2)*cos(e(2)/2)*sin(e(3)/2);
    x4 = cos(e(1)/2)*cos(e(2)/2)*sin(e(3)/2) - sin(e(1)/2)*sin(e(2)/2)*cos(e(3)/2);
    q = [x1;x2;x3;x4];
end