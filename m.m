function [x] = m(a,b)
    % x=[cos(b)*sin(a),-sin(b),cos(a)*cos(b)]';
  x=[sin(b)*cos(a),sin(b)*sin(a),cos(b)]';
end