# ------------------------------------------------------------------------
# Main Driver Script for Spring-Pendulum MBD Simulation
# ------------------------------------------------------------------------
# Includes modules, runs simulation, generates plots/animations
# ------------------------------------------------------------------------

include("src/constraints.jl")
include("src/dynamics.jl")
include("src/visualize.jl")


using .Constraints
using .Dynamics
using .Visualize
using Plots

default(legendfontsize=10)

# ------------------------------------------------------------------------
# Main Simulation
# ------------------------------------------------------------------------

function main()

    # ----- Simulation Parameters ----------------------------------------
    theta2_0 = 0.0       # initial bar angle (rad)
    x1_0     = 0.0       # initial block position (m)
    t_end    = 5.0       # total simulation time (s)
    N        = 300       # number of output time steps for plots/animation
    alpha    = 5.0       # Baumgarte stabilization parameter α
    beta     = 5.0       # Baumgarte stabilization parameter β

    println("=" ^ 100)
    println("ME 5180 | Project 03 | Spring-Pendulum Multibody Dynamics")
    println("=" ^ 100)
    println("Bar Length:          L  = $(L) m")
    println("Block Mass:          m₁ = $(m1) kg")
    println("Bar Mass:            m₂ = $(m2) kg")
    println("Spring Stiffness:    k  = $(k) N/m")
    println("Gravity:             g  = $(g) m/s²")
    println("Bar Inertia:         I₂ = $(round(I2, digits=6)) kg⋅m²")
    println("Initial Bar Angle:   θ₂ = $(theta2_0) rad")
    println("Initial Block Pos:   x₁ = $(x1_0) m")
    println("Simulation Time:     t  = $(t_end) s")
    println("Baumgarte α, β:      $(alpha), $(beta)")
    println("=" ^ 100)

    # ----- Create Results Directory (if it doesnt exist) ----------------
    mkpath("results")

    # ----- Run Dynamic Simulation ---------------------------------------
    println("\nRunning Augmented MBD simulation...")
    solution, q0 = run_dynamics(
        theta2_0 = theta2_0,
        x1_0     = x1_0,
        t_end    = t_end,
        alpha    = alpha,
        beta     = beta
    )
    println("Simulation complete.")

    # ----- Print Initial Conditions -------------------------------------
    println("\nInitial conditions:")
    labels = ["x₁", "y₁", "θ₁", "x₂", "y₂", "θ₂"]
    for i in 1:6
        println("  $(labels[i]) = $(round(q0[i], digits=6))")
    end

    # ----- Create Uniform Time Vector for Output -----------------------
    t_steps = collect(LinRange(0.0, t_end, N))

    # ----- Verify Constraint Satisfaction -------------------------------
    max_residual = 0.0
    for i in eachindex(t_steps)
        qi = solution(t_steps[i])[1:6]                # extract positions at time t
        C_val = C_eqs(qi, t_steps[i])                 # evaluate constraint equations
        residual = sqrt(sum(C_val .^ 2))              # Euclidean norm of residual
        max_residual = max(max_residual, residual)    # update max
    end
    println("  Max constraint residual: $(round(max_residual, sigdigits=4))")

    if max_residual < 1e-6
        println("  Constraints satisfied to high precision.")
    else
        println("  WARNING: constraint drift detected. Consider increasing α, β.")
    end

    # ----- Compute Lagrange Multipliers (Constraint Forces) -------------
    println("\nComputing constraint forces (Lagrange multipliers)...")
    lambda_all = compute_lambda(solution, t_steps)
    println("  Done. Extracted $(size(lambda_all, 2)) multipliers at $(size(lambda_all, 1)) time steps.")

    # ----- Compute Accelerations ----------------------------------------
    println("\nComputing accelerations...")
    ddq_all = compute_accelerations(solution, t_steps)

    # ----- Generate Static Plots ----------------------------------------
    println("\nGenerating plots...")

    # positions vs time
    plot_positions(t_steps, solution, filename="results/positions_vs_time.png")

    # velocities vs time
    plot_velocities(t_steps, solution, filename="results/velocities_vs_time.png")

    # accelerations vs time
    plot_accelerations(t_steps, ddq_all, filename="results/accelerations_vs_time.png")

    # constraint forces vs time
    plot_constraint_forces(t_steps, lambda_all, filename="results/constraint_forces.png")

    # constraint residual vs time
    plot_constraint_residual(t_steps, solution, filename="results/constraint_residual.png")

    # ----- Generate Animations ------------------------------------------
    println("\nGenerating animations...")
    animate_mechanism(t_steps, solution, filename="results/mechanism.gif", fps=30)
    animate_mechanism_forces(t_steps, solution, lambda_all, filename="results/mechanism_forces.gif", fps=30)

    # ----- Summary ------------------------------------------------------
    println("\n" * "=" ^ 100)
    println("All outputs saved to results/ directory:")
    println("     Static Plots:")
    println("          - positions_vs_time.png")
    println("          - velocities_vs_time.png")
    println("          - accelerations_vs_time.png")
    println("          - constraint_forces.png")
    println("          - constraint_residual.png")
    println("          - energy_vs_time.png")
    println("          - force_snapshot.png")
    println("     Animations:")
    println("          - mechanism.gif")
    println("          - mechanism_forces.gif")
    println("=" ^ 100)

end

# ------------------------------------------------------------------------
# Main Driver Call
# ------------------------------------------------------------------------

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end