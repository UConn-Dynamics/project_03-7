module Constraints

# ----------------------------------------------------------------
# Physical Constants
# ----------------------------------------------------------------

const L      = 0.40      # rigid bar length (m)
const half_L = L / 2     # half-length of the bar (m)
const m1     = 0.1       # mass of block 1 (kg)
const m2     = 0.3       # mass of rigid bar (kg)
const k      = 10.0      # spring stiffness (N/m)
const g      = 9.81      # gravitational acceleration (m/s^2)

# ----------------------------------------------------------------
# Moments of Inertia
# ----------------------------------------------------------------

const block_w = 0.10    # block width for visualization (m)
const block_h = 0.05    # block height for visualization (m)
const I1 = m1 * (block_w^2 + block_h^2) / 12   # moment of inertia of block (kg*m^2)

const I2 = m2 * L^2 / 12   # moment of inertia of bar (kg*m^2)

# ----------------------------------------------------------------
# Mass Matrix (6x6 diagonal)
# ----------------------------------------------------------------

"""
    mass_matrix()

Construct the 6x6 diagonal mass matrix for the system.
"""
function mass_matrix()

    return [m1   0.0  0.0  0.0  0.0  0.0;
            0.0  m1   0.0  0.0  0.0  0.0;
            0.0  0.0  I1   0.0  0.0  0.0;
            0.0  0.0  0.0  m2   0.0  0.0;
            0.0  0.0  0.0  0.0  m2   0.0;
            0.0  0.0  0.0  0.0  0.0  I2 ]

end

# ----------------------------------------------------------------
# Constraint Equations: →C(→q, t) = →0
# ----------------------------------------------------------------

"""
    C_eqs(q, t)

Evaluate the 4 constraint equations.

# Arguments
- `q`: generalized coordinate vector [x₁, y₁, θ₁, x₂, y₂, θ₂]
- `t`: current time

# Returns
- 4-element vector; each element should be zero when constraints are satisfied
"""
function C_eqs(q, t)

    # unpack generalized coordinates
    x1     = q[1]    # x-position of block center
    y1     = q[2]    # y-position of block center
    theta1 = q[3]    # orientation of block
    x2     = q[4]    # x-position of bar center
    y2     = q[5]    # y-position of bar center
    theta2 = q[6]    # orientation of bar

    return [
        y1,                                     # C₁: sliding joint (position)
        theta1,                                 # C₂: sliding joint (orientation)
        x1 - x2 + half_L * cos(theta2),         # C₃: pin joint x-component
        y1 - y2 + half_L * sin(theta2)          # C₄: pin joint y-component
    ]

end

# ----------------------------------------------------------------
# Wrapper for NonlinearSolve
# ----------------------------------------------------------------

"""
    C_nonlinear(q, t)

Wrapper around C_eqs for use with NonlinearSolve, which expects the signature f(u, p) where u is the unknown vector and p is a parameter. Here p = t (time).
"""
function C_nonlinear(q, t)

    return C_eqs(q, t)

end

# ----------------------------------------------------------------
# Jacobian: C_q = dC/dq
# ----------------------------------------------------------------

"""
    Cq_jacobian(q)

Compute the 4x6 constraint Jacobian matrix.

# Arguments
- 'q': generalized coordinate vector

# Returns
- 4x6 matrix of partial derivatives
"""
function Cq_jacobian(q)

    theta2 = q[6]    # only θ₂ appears in the constraints

    return [
        0.0  1.0  0.0   0.0   0.0   0.0                    ;
        0.0  0.0  1.0   0.0   0.0   0.0                    ;
        1.0  0.0  0.0  -1.0   0.0  -half_L*sin(theta2)     ;
        0.0  1.0  0.0   0.0  -1.0   half_L*cos(theta2)      
    ]

end

# ----------------------------------------------------------------
# Acceleration RHS: Q_d
# ----------------------------------------------------------------

"""
    Qd_accel(q, dq)

Compute the RHS vector (Q_d) of the acceleration constraint equation.

# Arguments
- 'q':  generalized coordinate vector
- 'dq': generalized velocity vector 

# Returns
- 4-element vector, Q_d
"""
function Qd_accel(q, dq)

    theta2  = q[6]     # bar orientation
    dtheta2 = dq[6]    # bar angular velocity

    return [
        0.0,                             
        0.0,                             
        half_L * cos(theta2) * dtheta2^2,
        half_L * sin(theta2) * dtheta2^2 
    ]

end

# ----------------------------------------------------------------
# External Force Vector: Q_e
# ----------------------------------------------------------------

"""
    Qe_forces(q)

Compute generalized external force vector, Q_e.

# Arguments
- 'q':  generalized coordinate vector

# Returns
- 6-element vector of generalized external forces, Q_e
"""
function Qe_forces(q)

    x1 = q[1]    # block x-position (determines spring stretch)

    return [
        -k * x1,        # Q₁: spring force on block (x-direction)
        -m1 * g,        # Q₂: gravity on block (y-direction)
         0.0,           # Q₃: no external torque on block (forces act on COM)
         0.0,           # Q₄: no external force on bar (x-direction)
        -m2 * g,        # Q₅: gravity on bar (y-direction)
         0.0            # Q₆: no external torque on bar (forces act on COM)
    ]

end

# ----------------------------------------------------------------
# Exported Functions/Parameters
# ----------------------------------------------------------------

export C_eqs, C_nonlinear, Cq_jacobian, Qd_accel, Qe_forces, mass_matrix
export L, half_L, m1, m2, k, g, I1, I2, block_w, block_h

end