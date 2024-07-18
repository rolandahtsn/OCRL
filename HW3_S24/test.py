#####################################################################################
#Helper Functions
###################################################################################

# #Get Quadrotor cost function
function compute_quad_cost(params::NamedTuple, Z::Vector)::Real
    idx, N, xg = params.idx, params.N, params.xg
    Q, R, Qf = params.Q, params.R, params.Qf
    
    # TODO: input cartpole LQR cost 
    
    J = 0 
    for i = 1:(N-1)
        xi = Z[idx.x[i]]
        ui = Z[idx.u[i]]
        x_gi = transpose(xi-xg)*Q*(xi-xg) 
        J += 0.5*x_gi + transpose(ui)*R*ui
    end
    # dont forget terminal cost 
    xN = Z[idx.x[N]]
    x_gN = transpose(xN-xg)*Qf*(xN-xg)
    J += 0.5*x_gN
    return J 
end

# #Use Hermite Simpson function
# function hermite_simpson(params::NamedTuple, x1::Vector, x2::Vector, u, dt::Real)::Vector
#     # TODO: input hermite simpson implicit integrator residual 
#     x_m = 0.5*(x1+x2) + (dt/8)*(combined_dynamics(params, x1, u) - combined_dynamics(params, x2,u))
#     xk_dot = combined_dynamics(params,x_m,u)
#     res = x1 + dt .* (combined_dynamics(params,x1,u)+4*xk_dot + combined_dynamics(params, x2, u))/6 - x2
#     return res
# end

# #Construct Constraints
# function quad_dynamic_contraints(params::NamedTuple, Z::Vector)::Vector
#     idx, N, dt = params.idx, params.N, params.dt
    
#     # TODO: create dynamics constraints using hermite simpson 
    
#     # create c in a ForwardDiff friendly way (check HW0)
#     c = zeros(eltype(Z), idx.nc)
    
#     for i = 1:(N-1)
#         xi = Z[idx.x[i]]
#         ui = Z[idx.u[i]] 
#         xip1 = Z[idx.x[i+1]]
        
#         # TODO: hermite simpson 
#         # c[idx.c[i]] = zeros(4)
#         c[idx.c[i]] = hermite_simpson(params,xi,xip1,ui,dt)
#     end
#     return c 
# end

# function quad_equality_constraint(params::NamedTuple, Z::Vector)::Vector
#     idx, N, dt = params.idx, params.N, params.dt

#     x0 = Z[idx.x[1]]
#     xN = Z[idx.x[N]]
#     xic = [params.x1ic; params.x2ic;params.x3ic]
#     xg = [params.x1g;params.x2g;params.x3g]
#     # c = zeros(eltype(Z), idx.nc)
#     c = quad_dynamic_contraints(params, Z)
#     res = [x0-xic; xN-xg; c]
#     # for i = 1:(N-1)
#     #     xi = Z[idx.x[i]]
#     #     ui = Z[idx.u[i]] 
#     #     xip1 = Z[idx.x[i+1]]
        
#     #     # integrate dynamics using Hermite-Simpson
#     #     x_m = 0.5 * (xi + xip1) + (dt / 8) * (single_quad_dynamics(params, xi, ui) - single_quad_dynamics(params, xip1, ui))
#     #     x_dot = single_quad_dynamics(params, x_m, ui)
#     #     c[idx.c[i]] = xi + dt * (single_quad_dynamics(params, xi, ui) + 4 * x_dot + single_quad_dynamics(params, xip1, ui)) / 6 - xip1
#     # end
    
#     return res
# end

# function quad_inequality_constraint(params::NamedTuple, Z::Vector)::Vector
#     idx, N, dt = params.idx, params.N, params.dt
    
#     # create c in a ForwardDiff friendly way (check HW0)
#     c = zeros(eltype(Z), 3 * (N - 1))
#     xg = params.xg
#     for i = 1:(N-1)
#         xi = Z[idx.x[i]]
#         xj = Z[idx.x[i + 1]]
        
#         # Euclidean distance between quadrotors i and j
#         dist_ij_squared = (xi[1] - xj[1])^2 + (xi[2] - xj[2])^2
        
#         # inequality constraint: distance between quadrotors should be greater than 0.8 meters
#         c[3 * (i - 1) + 1] = 0.8^2 - dist_ij_squared
#     end
    
#     return c
# end

function quadrotor_cost(params::NamedTuple, Z::Vector)::Real
    idx, N, xg = params.idx, params.N, params.xg
    Q, R, Qf = params.Q, params.R, params.Qf
    
    # input cartpole LQR cost 
    
    J = 0 
    for i = 1:(N-1)
        xi = Z[idx.x[i]]
        ui = Z[idx.u[i]]
        
        J += 0.5 * (xi - xg)' * Q * (xi - xg) + 0.5 * ui' * R * ui
    end
    
    # dont forget terminal cost 
    J += 0.5 * (Z[idx.x[N]] - xg)' * Qf * (Z[idx.x[N]] - xg)
    
    return J 
end
function quadrotor_dynamics_constraints(params::NamedTuple, Z::Vector)::Vector
    idx, N, dt = params.idx, params.N, params.dt
    
    # create dynamics constraints using hermite simpson 
    
    # create c in a ForwardDiff friendly way (check HW0)
    c = zeros(eltype(Z), idx.nc)
    
    for i = 1:(N-1)
        xi = Z[idx.x[i]]
        ui = Z[idx.u[i]] 
        xiplus1 = Z[idx.x[i+1]]
        
        # hermite simpson 
        c[idx.c[i]] = hermite_simpson(params, xi, xiplus1, ui, dt)
    end
    return c 
end

function quadrotor_equality_constraint(params::NamedTuple, Z::Vector)::Vector
    N, idx, xic, xg = params.N, params.idx, params.xic, params.xg 
    
    # return all of the equality constraints 
    
    return [Z[idx.x[1]] - xic; 
            Z[idx.x[N]] - xg;  # TODO change this to a inequality condt
            quadrotor_dynamics_constraints(params, Z)]
end
function quadrotor_inequality_constraint(params::NamedTuple, Z::Vector)::Vector
    N, idx, xic = params.N, params.idx, params.xic
    
    c = zeros(eltype(Z), 3 , N-1)
    
    for i = 1:(N-1)
        xi1 = Z[idx.x[i]][1:2]
        xi2 = Z[idx.x[i]][7:8]
        xi3 = Z[idx.x[i]][13:14]
        c[:,i] = [norm(xi1-xi2)^2;norm(xi2-xi3)^2;norm(xi3-xi1)^2]
    end
    return c[:]
end
function create_idx(nx,nu,N)
    # This function creates some useful indexing tools for Z 
    # x_i = Z[idx.x[i]]
    # u_i = Z[idx.u[i]]
    
    # Feel free to use/not use anything here.
    
    
    # our Z vector is [x0, u0, x1, u1, …, xN]
    nz = (N-1) * nu + N * nx # length of Z 
    x = [(i - 1) * (nx + nu) .+ (1 : nx) for i = 1:N]
    u = [(i - 1) * (nx + nu) .+ ((nx + 1):(nx + nu)) for i = 1:(N - 1)]
    
    # constraint indexing for the (N-1) dynamics constraints when stacked up
    c = [(i - 1) * (nx) .+ (1 : nx) for i = 1:(N - 1)]
    nc = (N - 1) * nx # (N-1)*nx 
    
    return (nx=nx,nu=nu,N=N,nz=nz,nc=nc,x= x,u = u,c = c)
end

"""
    quadrotor_reorient

Function for returning collision free trajectories for 3 quadrotors. 

Outputs:
    x1::Vector{Vector}  # state trajectory for quad 1 
    x2::Vector{Vector}  # state trajectory for quad 2 
    x3::Vector{Vector}  # state trajectory for quad 3 
    u1::Vector{Vector}  # control trajectory for quad 1 
    u2::Vector{Vector}  # control trajectory for quad 2 
    u3::Vector{Vector}  # control trajectory for quad 3 
    t_vec::Vector
    params::NamedTuple

The resulting trajectories should have dt=0.2, tf = 5.0, N = 26
where all the x's are length 26, and the u's are length 25. 

Each trajectory for quad k should start at `xkic`, and should finish near 
`xkg`. The distances between each quad should be greater than 0.8 meters at 
every knot point in the trajectory. 
"""
# function quadrotor_reorient(;verbose=true)
    
#     # problem size 
#     nx = 18 
#     nu = 6
#     dt = 0.2
#     tf = 5.0 
#     t_vec = 0:dt:tf 
#     N = length(t_vec)
    
#     # indexing 
#     idx = create_idx(nx,nu,N)
    
#     # initial conditions and goal states 
#     lo = 0.5 
#     mid = 2 
#     hi = 3.5 
#     x1ic = [-2,lo,0,0,0,0]  # ic for quad 1 
#     x2ic = [-2,mid,0,0,0,0] # ic for quad 2 
#     x3ic = [-2,hi,0,0,0,0]  # ic for quad 3 
#     xic = [x1ic; x2ic; x3ic]
#     x1g = [2,mid,0,0,0,0]   # goal for quad 1 
#     x2g = [2,hi,0,0,0,0]    # goal for quad 2 
#     x3g = [2,lo,0,0,0,0]    # goal for quad 3 
#     xg = [x1g; x2g; x3g]
    
#     Q = diagm(ones(idx.nx))
#     R = 0.1*diagm(ones(idx.nu))
#     Qf = 10*diagm(ones(idx.nu))
#     # load all useful things into params 
#     # TODO: include anything you would need for a cost function (like a Q, R, Qf if you were doing an 
#     # LQR cost)
#     params = (x1ic=x1ic,
#               x2ic=x2ic,
#               x3ic=x3ic,
#               x1g = x1g,
#               x2g = x2g,
#               x3g = x3g,
#               xic = xic,
#               xg = xg,
#               dt = dt,
#               N = N,
#               idx = idx,
#               mass = 1.0, # quadrotor mass 
#               g = 9.81,   # gravity 
#               ℓ = 0.3,    # quadrotor length 
#               J = .018,   # quadrotor moment of inertia 
#               Q = Q,
#               Qf = Qf,
#               R = R)   
    
#     # TODO: solve for the three collision free trajectories however you like
#     idx = params.idx
#     nu = idx.nu
#     nx = idx.nx
     
#     # TODO: primal bounds 
#     # you may use Inf, like Inf*ones(10) for a vector of positive infinity 
#     x_l = -Inf*ones(idx.nz)
#     x_u = Inf*ones(idx.nz)

#     # TODO: inequality constraint bounds 
#     c_l = 0.64*ones(3*(N-1))
#     c_u = Inf*ones(3*(N-1)) 

#     #initial guess
#     z0 = zeros(idx.nz)
#     x0 = range(xic,xg, length=N)

#     for i=1:(N-1)
#         z0[idx.x[i]] = x0[i]
#     end

#     diff_type = :auto

#     # Z = fmincon(compute_quad_cost,quad_equality_constraint,quad_inequality_constraint,
#     #             x_l,x_u,c_l,c_u,z0,params, diff_type;
#     #             tol = 1e-6, c_tol = 1e-6, max_iters = 10_000, verbose = verbose)

#     Z = fmincon(quadrotor_cost,quadrotor_equality_constraint,quadrotor_inequality_constraint,
#                     x_l,x_u,c_l,c_u,z0,params, diff_type;
#                     tol = 1e-6, c_tol = 1e-6, max_iters = 10_000, verbose = verbose)

#     # pull the X and U solutions out of Z 
#     X = [Z[idx.x[i]] for i = 1:N]
#     U = [Z[idx.u[i]] for i = 1:(N-1)]

#     # return the trajectories 
#     x1 = [X[i][1:6] for i=1:N]
#     x2 = [X[i][7:12] for i=1:N]
#     x3 = [X[i][13:18] for i=1:N]
#     u1 = [U[i][1:2] for i=1:(N-1)]
#     u2 = [U[i][3:4] for i=1:(N-1)]
#     u3 = [U[i][5:6] for i=1:(N-1)]
        
#     return x1, x2, x3, u1, u2, u3, t_vec, params 
# end

function quadrotor_reorient(;verbose=true)
    
    # problem size 
    nx = 18 
    nu = 6
    dt = 0.2
    tf = 5.0 
    t_vec = 0:dt:tf 
    N = length(t_vec)
    
    # indexing 
    idx = create_idx(nx,nu,N)
    
    # initial conditions and goal states 
    lo = 0.5 
    mid = 2 
    hi = 3.5 
    x1ic = [-2,lo,0,0,0,0]  # ic for quad 1 
    x2ic = [-2,mid,0,0,0,0] # ic for quad 2 
    x3ic = [-2,hi,0,0,0,0]  # ic for quad 3 
    xic = [x1ic; x2ic; x3ic]
    x1g = [2,mid,0,0,0,0]   # goal for quad 1 
    x2g = [2,hi,0,0,0,0]    # goal for quad 2 
    x3g = [2,lo,0,0,0,0]    # goal for quad 3 
    xg = [x1g; x2g; x3g]
    # load all useful things into params 
    # TODO: include anything you would need for a cost function (like a Q, R, Qf if you were doing an 
    # LQR cost)
    
    Q = diagm(ones(nx))
    R = 0.1*diagm(ones(nu))
    Qf = 10*diagm(ones(nx))
    separation_radius = 0.8
    
    params = (x1ic=x1ic,
              x2ic=x2ic,
              x3ic=x3ic,
              x1g = x1g,
              x2g = x2g,
              x3g = x3g,
#         ------------------------
              xic = xic,
              xg = xg,
              Q = Q,
              Qf = Qf,
              R = R,       
#         ------------------------
              dt = dt,
              N = N,
              idx = idx,
              mass = 1.0, # quadrotor mass 
              g = 9.81,   # gravity 
              ℓ = 0.3,    # quadrotor length 
              J = .018)   # quadrotor moment of inertia 
    
    # primal bounds 
    x_l = -Inf * ones(idx.nz)
    x_u = Inf * ones(idx.nz)
    
    # solve for the three collision free trajectories however you like
    
    # inequality constraint bounds (this is what we do when we have no inequality constraints)
    c_l = separation_radius^2 * ones( 3 * (idx.N-1))
    c_u = Inf * ones(3 * (idx.N-1))
    
    # initial guess 
    z0 = 0.001*randn(idx.nz)
    x_initial_guess = range(xic, xg, length = N)
    for i = 1:(N)
        z0[idx.x[i]] = x_initial_guess[i]
    end
    
    diff_type = :auto 

    Z = fmincon(quadrotor_cost,quadrotor_equality_constraint,quadrotor_inequality_constraint,
                x_l,x_u,c_l,c_u,z0,params, diff_type;
                tol = 1e-6, c_tol = 1e-6, max_iters = 10_000, verbose = verbose)
    
    # return the trajectories 
    
    x1 = [Z[idx.x[i]][1:6]  for i = 1:N]
    x2 = [Z[idx.x[i]][7:12]  for i = 1:N]
    x3 = [Z[idx.x[i]][13:18]  for i = 1:N]
    u1 = [Z[idx.u[i]][1:2]  for i = 1:(N-1)]
    u2 = [Z[idx.u[i]][3:4]  for i = 1:(N-1)]
    u3 = [Z[idx.u[i]][5:6]  for i = 1:(N-1)]
        
    return x1, x2, x3, u1, u2, u3, t_vec, params 
end