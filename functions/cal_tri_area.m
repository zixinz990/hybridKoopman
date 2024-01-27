function area = cal_tri_area(p1, p2, p3)
area = 0.5 * abs(p1(1)*(p2(2) - p3(2))+p2(1)*(p3(2) - p1(2))+p3(1)*(p1(2) - p2(2)));
end