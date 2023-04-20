%%%%%%%%%%%%% Actuation Matrix %%%%%%%%%%%%%%
function Btau = actuationMatrix(xi, actuation_path, X_sym, X_des)
    X = X_sym;
    
    %% Extract Number of Actuators
    [~, na] = size(actuation_path);
    
    if(nargin > 3)
        %% Compute Actuation Matrix
        for i=1:na
            % Symbolic Computation
            t = vec6D2se3(xi)*[actuation_path(:, i); 1] + diff([actuation_path(:, i); 1], X);
            % Subs Block
            d = subs(actuation_path(:, i), X, X_des);
            t = subs(t, X, X_des);
            % Norm
            norm_t = simplify(t(1:end-1)/(norm(t(1:end-1))));
            % Actuation Matrix
            Btau(:, i) = simplify([skew(d)*norm_t; norm_t]);
        end
    else
        %% Compute Actuation Matrix
        for i=1:na
            % Symbolic Computation
            t = vec6D2se3(xi)*[actuation_path(:, i); 1] + diff([actuation_path(:, i); 1], X);

            % Norm
            norm_t = simplify(t(1:end-1)/(norm(t(1:end-1))));
            % Actuation Matrix
            Btau(:, i) = simplify([skew(actuation_path(:, i))*norm_t; norm_t]);
        end
    end
end