function [e, a1, a2, a3, a4, K] = HeadingErrorDynamics(r_fd,r,v,g)
    R = r_fd - r;
    R_dot = -v;

    e = acos((dot(R,v))/(norm(R)*norm(v)));
    var = (norm(R)*norm(v))^2 * (1-((dot(R,v))/(norm(R)*norm(v)))^2)^0.5 ;
    K = -(1/var);

    a1 = K*(norm(R)*norm(v)*(r_fd(1) - r(1)) - ((dot(R,v)*norm(R)*v(1))/norm(v)));
    a2 = K*(norm(R)*norm(v)*(r_fd(2) - r(2)) - ((dot(R,v)*norm(R)*v(2))/norm(v)));
    a3 = K*(norm(R)*norm(v)*(r_fd(3) - r(3)) - ((dot(R,v)*norm(R)*v(3))/norm(v)));
    a4 = K*(norm(R)*norm(v)*(dot(R,g)) - ((dot(R,v)*norm(R)*(dot(v,g)))/norm(v)) + norm(R)*norm(v)*(dot(R_dot,v)) - (((dot(R,v))*norm(v)*(dot(R,R_dot)))/(norm(R))));
end


