function plottrajandaccel(sol,p,p0,pF)
figure
psol = evaluate(p, sol);
plot3(psol(:,1),psol(:,2),psol(:,3),'rx')
hold on
plot3(p0(1),p0(2),p0(3),'ks')
plot3(pF(1),pF(2),pF(3),'bo')
hold off
view([18 -10])
xlabel("x")
ylabel("y")
zlabel("z")
legend("Steps","Initial Point","Final Point")
figure
asolm = sol.a;
nasolm = sqrt(sum(asolm.^2,2));
plot(nasolm,"rx")
xlabel("Time step")
ylabel("Norm(acceleration)")
end