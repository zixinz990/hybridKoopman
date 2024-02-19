function area = cal_tri_area(p1, p2, p3)
% Calculate the area of a triangle given the coordinate of vertices
area = 0.5 * abs(p1(1)*(p2(2) - p3(2))+p2(1)*(p3(2) - p1(2))+p3(1)*(p1(2) - p2(2)));
end