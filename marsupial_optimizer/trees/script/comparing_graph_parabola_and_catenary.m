clear;
clc;

cat=[-6.40000 2.00000 0.00000 -6.50000 3.00000 1.50000 -0.799 0.142 1.379]; %Posicion 36 catenary (coll car 36 - 38)
par=[4.00000 -0.08750 0.00000 1.10000 -0.95000 6.10000 1.111 -1.472 0.380];%Posicion 36 parabola

num_point_per_unit_length = 20;
delta_x = (cat(1,1) - cat(1,4));
delta_y = (cat(1,2) - cat(1,5));
delta_z = (cat(1,3) - cat(1,6));
dxy_ = sqrt(delta_x * delta_x + delta_y * delta_y );
dist_= sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z );
u_x = delta_x /dxy_;
u_y = delta_y /dxy_;
d_ = dxy_;
d_z = abs(cat(1,3)-cat(1,6));

if (dxy_ < 0.0001)
	u_x = 0.0;
    u_y = 0.0;
	d_ = d_z;
end
	
if (d_ < 1.0)
        if (d_ < abs(cat(1,3)-cat(1,3)))
            num_point_catenary = ceil(abs(cat(1,3)-cat(1,3)) * 10.0);
        else
            num_point_catenary = ceil(d_ * 10.0);
        end
else
	num_point_catenary = ceil(num_point_per_unit_length * d_);
end

v_pts_par=[]; v_pts_cat=[];
% Catenary Equation
step = 0;
for c = 0:num_point_catenary
    x = cat(1,1) + u_x * step;
    y = cat(1,2) + u_y * step;
    
    z_c = cat(1,9)*cosh((step-cat(1,7))/cat(1,9))+(cat(1,8)-cat(1,9));
    z_p = par(1,7)*step*step+par(1,8)*step+par(1,9); % Catenary equation

    step = step + d_/ num_point_catenary;  
    v_pts_cat=[v_pts_cat;x,y,z_c];
    v_pts_par=[v_pts_par;x,y,z_p];
end

plot3(v_pts_par(:,1), v_pts_par(:,2), v_pts_par(:,3),'b', 'DisplayName','Parabola')
hold on;
plot3(v_pts_cat(:,1), v_pts_cat(:,2), v_pts_cat(:,3),'r', 'DisplayName','Catenary')
legend
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis')
title('Parabola vs Catenary');
grid;