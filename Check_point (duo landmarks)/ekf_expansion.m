function [xiHat, Sigma, state_ids] = ekf_expansion(xiHat, Sigma, lms, ids, state_ids, R)
% function that expand the state vector and covariance after measuremnt if
% new landmark observers
% #inputs:
% xiHat: state vector before measurement
% Sigma: covariance before measurement
% lms: measured landmark position
% ids: measured landmark ids (corresponds with position index)
% #outputs:
% xiHat: expanded state vector with the new landmark information
% Sigma: expanded covariance matrix

    for i = 1:numel(ids)
       lm = lms(:,i);
       id = ids(i);
       
       % if the id is in our state_ids we don't need to add anything
       if any(id == state_ids)
           continue
       end
       
       % Augment
       % if the observed landmark is new, add to the state
       inertial_lm = convert2inertial(xiHat(1:3),lm);
       
       % compute G matrix for Sigma expansion
       G_xi_0 = [1,0,-sin(xiHat(3))*lm(1)-cos(xiHat(3))*lm(2);
                0,1,cos(xiHat(3))*lm(1)-sin(xiHat(3))*lm(2)];
       G_xi = [G_xi_0,zeros(2,numel(xiHat)-3)];
        
       G_z = [cos(xiHat(3)),-sin(xiHat(3));
              sin(xiHat(3)),cos(xiHat(3))];
          
       % expand the state when new landmark added
       state_ids = [state_ids, id];
       xiHat = [xiHat; inertial_lm];
       
       
       
       % expand Sigma
       Sigma = [Sigma, Sigma*G_xi';
                G_xi*Sigma, G_xi*Sigma*G_xi'+G_z*R*G_z'];
    end

end

