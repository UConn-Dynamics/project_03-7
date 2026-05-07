module Dynamics

using NonlinearSolve
using DifferentialEquations
using LinearAlgebra
using ..Constraints

# ----------------------------------------------------------------
# Number of Coordinates and Constraints
# ----------------------------------------------------------------

const n  = 6     # number of generalized coordinates (2 bodies × 3 each)
const nc = 4     # number of constraint equations

# ----------------------------------------------------------------
# Assemble the Augmented LHS Matrix
# ----------------------------------------------------------------

"""
    LHS_augmented(q)

Build the augmented LHS matrix for the MBD system.

# Arguments
- `q`: generalized coordinate vector

# Returns
- 10x10 matrix
"""
function LHS_augmented(q)

    M  = mass_matrix()       # 6×6 diagonal mass matrix
    Cq = Cq_jacobian(q)      # 4×6 constraint Jacobian

    return [M Cq';        
            Cq zeros(nc, nc)]

end

# ----------------------------------------------------------------
# Assemble the Augmented RHS Vector
# ----------------------------------------------------------------

"""
    RHS_augmented(q, dq)

Build the augmented RHS vector for the MBD system.

# Arguments
- 'q':  generalized coordinate vector
- 'dq': generalized velocity vector 

# Returns
- 10-element vector
"""
function RHS_augmented(q, dq)

    Qe = Qe_forces(q)        # 6 element external force vector
    Qd = Qd_accel(q, dq)     # 4 element acceleration constraint vector

    return [Qe; Qd]           # stack into a 10 element vector

end

# ----------------------------------------------------------------------------------
# ODE RHS Function (state derivative function required by DifferentialEquations.jl)
# ----------------------------------------------------------------------------------

"""
    mbd_ode!(dy, y, params, t)

# Computes dy/dt = [dq; ddq] by solving the augmented system.

The state vector is y = [q; q̇] (12 elements)
The derivative is ẏ = [q̇; q̈] (12 elements)

At each time step:
1. Extract positions q and velocities q̇ from the state
2. Build the augmented system
3. Apply Baumgarte stabilization to the constraint rows
4. Solve the 10x10 linear system
5. Pack q̇ and q̈ into ẏ   

# Arguments
- `dy`:     output derivative vector (12 elements), modified in-place
- `y`:      current state vector [q; q̇] (12 elements)
- `params`: Baumgarte parameters [α, β]
- `t`:      current time
"""
function mbd_ode!(dy, y, params, t)

    # ----- Step 1: Unpack State ----------------------------------------
    q  = y[1:n]        # generalized coordinates [x₁, y₁, θ₁, x₂, y₂, θ₂]
    dq = y[n+1:2*n]    # generalized velocities  [ẋ₁, ẏ₁, θ̇₁, ẋ₂, ẏ₂, θ̇₂]

    # ----- Step 2: Unpack Baumgarte Parameters -------------------------
    alpha, beta = params    # α and β for Baumgarte stabilization

    # ----- Step 3: Build Augmented System ------------------------------
    A   = LHS_augmented(q)          # 10×10 augmented matrix
    rhs = RHS_augmented(q, dq)      # 10 element RHS vector

    # ----- Step 4: Apply Baumgarte Stabilization -----------------------

    Cq = Cq_jacobian(q)                              # 4×6 constraint Jacobian
    C  = C_eqs(q, t)                                 # 4 element constraint residual

    rhs[n+1:n+nc] += -2 * alpha * Cq * dq - beta^2 * C   # add stabilization terms to Q_d

    # ----- Step 5: Solve the Augmented System --------------------------
    sols = A \ rhs              # 10 element solution vector

    ddq    = sols[1:n]          # extract accelerations (first 6 elements)
    # lambda = sols[n+1:n+nc]   # extract Lagrange multipliers (last 4, not needed for integration)

    # ----- Step 6: Pack Derivative Vector ------------------------------
    dy[1:n]       = dq          # velocities
    dy[n+1:2*n]   = ddq         # accelerations from the augmented solve)

    return nothing              # in-place modification, no return value needed

end

# ----------------------------------------------------------------
# Solve for Initial Positions (Nonlinear)
# ----------------------------------------------------------------

"""
    solve_initial_position(theta2_0, x1_0)

Find the initial generalized coordinate vector q₀ that satisfies all constraints, given the initial bar angle θ₂(0) and block position x₁(0).

# Arguments
- `theta2_0`: initial bar angle (rad), e.g., 0 for horizontal
- `x1_0`:     initial block x-position (m), e.g., 0 for spring natural length

# Returns
- 6 element initial generalized coordinate vector, q₀
"""
function solve_initial_position(theta2_0, x1_0)

    # define the nonlinear system: 4 constraints + 2 initial conditions = 6 equations
    function setup_eqs(q, p)

        theta2_init, x1_init = p     # unpack the desired initial values

        return [
            C_eqs(q, 0.0);           # 4 constraint equations at t = 0
            q[1] - x1_init;          # fix x₁ to specified initial value
            q[6] - theta2_init       # fix θ₂ to specified initial angle
        ]

    end

    # initial guess
    q_guess = [x1_0, 0.0, 0.0, x1_0 + half_L * cos(theta2_0), half_L * sin(theta2_0), theta2_0]

    # create and solve the nonlinear problem
    prob = NonlinearProblem(setup_eqs, q_guess, (theta2_0, x1_0))
    sol  = solve(prob)

    return Vector(sol.u)    # return as a plain vector

end

# ----------------------------------------------------------------
# Run Full Dynamic Simulation
# ----------------------------------------------------------------

"""
    run_dynamics(; theta2_0=0.0, x1_0=0.0, t_end=5.0, alpha=5.0, beta=5.0)

Run the full augmented MBD simulation.

# Keyword Arguments
- `theta2_0`: initial bar angle (rad), default 0.0 (horizontal)
- `x1_0`:     initial block x-position (m), default 0.0 (spring natural length)
- `t_end`:    total simulation time (s), default 5.0
- `alpha`:    Baumgarte stabilization parameter α, default 5.0
- `beta`:     Baumgarte stabilization parameter β, default 5.0

# Returns
- `solution`: ODE solution object
- `q0`:       initial position vector
"""
function run_dynamics(; theta2_0=0.0, x1_0=0.0, t_end=5.0, alpha=5.0, beta=5.0)

    # ----- Step 1: Solve for Initial Positions -------------------------
    q0 = solve_initial_position(theta2_0, x1_0)

    # ----- Step 2: Set Initial Velocities to Zero (released from rest) -
    dq0 = zeros(n)

    # ----- Step 3: Assemble Initial State Vector -----------------------
    y0 = [q0; dq0]    # 12-element state: [→q₀; →q̇₀]

    # ----- Step 4: Define and Solve the ODE ----------------------------
    time_span = (0.0, t_end)                                    # simulation interval
    params    = [alpha, beta]                                   # Baumgarte parameters
    problem   = ODEProblem(mbd_ode!, y0, time_span, params)     # define the ODE problem
    solution  = solve(problem, Tsit5())                         # solve using Tsit5

    return solution, q0

end

# -------------------------------------------------------------------
# Extract Lagrange Multipliers (Constraint Forces) at Each Time Step
# -------------------------------------------------------------------

"""
    compute_lambda(solution, t_steps)

Re-solve the augmented system at each output time step to extract the Lagrange multipliers.

# Arguments
- `solution`: ODE solution object
- `t_steps`:  vector of time values at which to evaluate

# Returns
- `lambda_all`: Nx4 matrix of Lagrange multipliers at each time step
"""
function compute_lambda(solution, t_steps)

    N = length(t_steps)                 # number of output time steps
    lambda_all = zeros(N, nc)           # storage for Lagrange Multiplier values (N × 4)

    for i in 1:N

        t  = t_steps[i]                 # current time
        yi = solution(t)                # interpolate state at this time

        q  = yi[1:n]                    # extract positions
        dq = yi[n+1:2*n]                # extract velocities

        A   = LHS_augmented(q)          # build 10×10 augmented matrix
        rhs = RHS_augmented(q, dq)      # build 10-element RHS (without Baumgarte)

        sols = A \ rhs                  # solve the system

        lambda_all[i, :] = sols[n+1:n+nc]    # extract Lagrange multipliers

    end

    return lambda_all

end

# ----------------------------------------------------------------
# Exported Functions/Parameters
# ----------------------------------------------------------------

export run_dynamics, compute_lambda
export LHS_augmented, RHS_augmented, mbd_ode!
export n, nc

end