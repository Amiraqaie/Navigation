    syms lon lat h vn ve vd phi theta si real
    syms wx wy wz fx fy fz real
    syms Ts real
    
    x_sym = [lon; lat; h; vn; ve; vd; phi; theta; si];
    Wbi_b_sym = [wx; wy; wz];
    Fbi_b_sym = [fx; fy; fz];

    dx1 = state_derivative(x_sym, Wbi_b_sym, Fbi_b_sym) * Ts;
    
    x_next = x_sym + dx1;
    
    % state Jacobian
    Fk_sym = jacobian(x_next, x_sym);

    Bk_sym = jacobian(x_next, [Wbi_b_sym; Fbi_b_sym]);

    % Create function file for numeric Jacobian evaluation
    matlabFunction(Fk_sym, 'File', 'Fk', ...
        'Vars', {x_sym, Wbi_b_sym, Fbi_b_sym, Ts}); 

    matlabFunction(Bk_sym, 'File', 'Bk', ...
        'Vars', {x_sym, Wbi_b_sym, Fbi_b_sym, Ts});
