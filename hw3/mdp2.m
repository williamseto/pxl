clear
%%% policy iteration

gamma = 0;

T = [0.05, 0.05, 0.9;
     0.25, 0.50, 0.25;
     0.01, 0.02, 0.97];
     
pol = [3, 2, 1];

R = [1; -1; 0];


policy_stable = 0;

count = 0;
count2 = 0
while ~policy_stable

  policy_stable = 1;
  count = count + 1;

  % solve value function (policy evaluation)

  P = [T(pol(1),:); T(pol(2),:); T(pol(3),:)];
  v = (gamma*P - eye(3)) \ (-R);
  
  eps = 0.001;
  curr_eps = [999 999 999];
  V = [0 0 0];
  while any(curr_eps > eps)

    count2 = count2 + 1;
    tmp_V = [0 0 0];
    for i=1:3
      %tmp = R(i);
      tmp = 0;
      for j=1:3
        %tmp = tmp + T(i,j)*gamma*V(j);
        tmp = tmp + P(i,j)*(R(i) + gamma*V(j));
      end   
      curr_eps(i) = abs(V(i) - tmp);
      tmp_V(i) = tmp;
    end
    V = tmp_V;
  end 
  v = V;

  % policy improvement
  
  tmp_pol = [0 0 0];
  %for each state
  for s=1:3
    best_q = v(s);
    
    % each action
    for a=1:3
        
      tmp_q = R(s);
      % for each successor
      for j=1:3
        tmp_q = tmp_q + T(a,j)*gamma*v(j);
      end
      
      if tmp_q > best_q
        pol(s) = a;
        best_q = tmp_q;
        policy_stable = 0;
      end
    
    
    end
  
  end
  
end

pol
v
count
count2






