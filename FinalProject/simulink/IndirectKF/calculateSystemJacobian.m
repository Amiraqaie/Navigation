    syms lon lat h vn ve vd phi theta si real
    syms wx wy wz fx fy fz real
    syms bax bay baz
    syms bgx bgy bgz
    syms Ts
    
    x_sym = [lon; lat; h; vn; ve; vd; phi; theta; si; bax; bay; baz; bgx; bgy; bgz];
    Wbi_b_sym = [wx; wy; wz];
    Fbi_b_sym = [fx; fy; fz];
    
    dx1 = state_derivative(x_sym, Wbi_b_sym, Fbi_b_sym) * Ts;
    
    dx2 = state_derivative(x_sym + dx1 / 2, Wbi_b_sym, Fbi_b_sym) * Ts;
    
    dx3 = state_derivative(x_sym + dx2 / 2, Wbi_b_sym, Fbi_b_sym) * Ts;
    
    dx4 = state_derivative(x_sym + dx3, Wbi_b_sym, Fbi_b_sym) * Ts;
    
    x_next = x_sym + (1/6) * (dx1 + 2 * dx2 + 2 * dx3 + dx4);
    
    Fk_sym = jacobian(x_next, x_sym);

    % input (noise) Jacobian
    Bk_sym = jacobian(x_next, [Wbi_b_sym; Fbi_b_sym]);
    
    % Create function file for numeric Jacobian evaluation
    matlabFunction(Fk_sym, 'File', 'Fk_numeric_simple', ...
        'Vars', {x_sym, Wbi_b_sym, Fbi_b_sym, Ts}); 

    matlabFunction(Bk_sym, 'File', 'Bk_numeric_simple', ...
               'Vars', {x_sym, Wbi_b_sym, Fbi_b_sym, Ts});