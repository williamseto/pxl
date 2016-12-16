
%%% value iteration

clear
rewards = [4 0 -8];
%rewards = [1 -1 0];

% T(i,j) = P(s(j) | s(i))

T = [0.5, 0.5, 0;
     0.5, 0, 0.5;
     0, 0.5, 0.5];
     
%T = [0.05, 0.05, 0.9;
%     0.05, 0.05, 0.9;
%     0.05, 0.05, 0.9];
     
V = [0 0 0];
gamma = 0.5;

eps = 0.001;
curr_eps = 999;
for n=1:1000
%while curr_eps > eps

  %for each state
  tmp_V = [0 0 0];
  for i=1:3
    %tmp = rewards(i);
    tmp = 0;
    % for each successor
    for j=1:3
      %tmp = tmp + T(i,j)*gamma*V(j);
      tmp = tmp + T(i,j)*(rewards(i) + gamma*V(j));
    end
    
    curr_eps = V(i) - tmp;
    tmp_V(i) = tmp;
  end
  V = tmp_V;

end

V