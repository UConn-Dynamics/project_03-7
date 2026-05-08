module Visualize

using Plots
using Measures
using LaTeXStrings

using ..Constraints: L, half_L, m1, m2, k, g, I1, I2, block_w, block_h
using ..Constraints: C_eqs
using ..Dynamics: n, nc

const track_len = 0.6

# ----------------------------------------------------------------
# Mechanism Drawing Helper: Block
# ----------------------------------------------------------------

"""
    draw_block!(p, q1; color=:steelblue)

Draw the sliding block (body 1) as a filled rectangle centered at (x₁, y₁) with orientation θ₁.

# Arguments
- `p`:     plot object
- `q1`:    [x₁, y₁, θ₁] --> position and orientation of block center
- `color`: fill color
"""
function draw_block!(p, q1; color=:steelblue)

    cx, cy, theta = q1                              # unpack center and angle

    hw = block_w / 2                                 # half-width
    hh = block_h / 2                                 # half-height

    # define corner coordinates in the body-fixed frame (counterclockwise)
    local_x = [-hw,  hw,  hw, -hw]
    local_y = [-hh, -hh,  hh,  hh]

    # transform to global coordinates using the rotation matrix A(θ)
    global_x = cx .+ cos(theta) .* local_x .- sin(theta) .* local_y
    global_y = cy .+ sin(theta) .* local_x .+ cos(theta) .* local_y

    # draw the filled rectangle
    block_shape = Shape(global_x, global_y)
    plot!(p, block_shape, c=color, linecolor=:black, alpha=0.6, label="")

end

# ----------------------------------------------------------------
# Mechanism Drawing Helper: Spring
# ----------------------------------------------------------------

"""
    draw_spring!(p, x_start, x_end, y_level; n_coils=8, amp=0.015)

Draw a zigzag spring between two x-positions at a given height of y.

# Arguments
- `p`:       plot object
- `x_start`: left attachment x-coordinate (wall)
- `x_end`:   right attachment x-coordinate (block center)
- `y_level`: y-coordinate for the spring centerline
- `n_coils`: number of coils to draw
- `amp`:     amplitude (vertical extent) of each coil
"""
function draw_spring!(p, x_start, x_end, y_level; n_coils=8, amp=0.015)

    n_pts  = 2 * n_coils + 2                         # total points in the zigzag
    xs     = LinRange(x_start, x_end, n_pts)         # evenly spaced x-coordinates
    ys     = zeros(n_pts)                            # y-coordinates start at zero

    for i in 2:n_pts-1                               # skip first and last (attachment points)
        ys[i] = y_level + amp * (-1)^i               # alternate up/down for zigzag pattern
    end

    ys[1]     = y_level                              # start at centerline
    ys[n_pts] = y_level                              # end at centerline

    plot!(p, collect(xs), ys, color=:black, lw=1.5, label="")   # draw the spring

end

# ----------------------------------------------------------------
# Mechanism Drawing Helper: Bar (Compound Pendulum)
# ----------------------------------------------------------------

"""
    draw_bar!(p, q2; color=:darkorange)

Draw the rigid bar (body 2) as a thick line from one end to the other.

# Arguments
- `p`:     plot object
- `q2`:    [x₂, y₂, θ₂] --> position and orientation of bar center
- `color`: line color
"""
function draw_bar!(p, q2; color=:darkorange)

    cx, cy, theta = q2                               # unpack center and angle

    # compute endpoint positions in global coordinates
    # End A (pin end), local coordinates = [-L/2, 0]
    x_A = cx - half_L * cos(theta)
    y_A = cy - half_L * sin(theta)

    # End B (free end), local coordinates = [L/2, 0]
    x_B = cx + half_L * cos(theta)
    y_B = cy + half_L * sin(theta)

    # draw the bar as a thick line
    plot!(p, [x_A, x_B], [y_A, y_B], color=color, lw=5, label="")

end

# ----------------------------------------------------------------
# Full Mechanism Drawing
# ----------------------------------------------------------------

"""
    draw_mechanism!(p, q)

Draw the complete two-body mechanism: track, spring, wall, block, bar, and joints.

# Arguments
- `p`: plot object
- `q`: generalized coordinate vector
"""
function draw_mechanism!(p, q)

    x1, y1, theta1 = q[1], q[2], q[3]     # block position and orientation
    x2, y2, theta2 = q[4], q[5], q[6]     # bar center position and orientation

    # ----- Draw Horizontal Track -----------------------------------
    #track_len = 0.6                         # half-length of track line
    plot!(p, [-track_len, track_len], [0.0, 0.0],
          color=:gray, lw=2, ls=:dash, label="")

    # ----- Draw Wall -----------------------------------------------
    wall_x = -track_len                     # wall is at the left end of the track
    plot!(p, [wall_x, wall_x], [-0.05, 0.05],
          color=:black, lw=3, label="")

    # ----- Draw Spring ---------------------------------------------
    draw_spring!(p, wall_x, x1 - block_w/2, y1)

    # ----- Draw Block (Body 1) ------------------------------------
    draw_block!(p, [x1, y1, theta1])

    # ----- Draw Bar (Body 2) --------------------------------------
    draw_bar!(p, [x2, y2, theta2])

    # ----- Draw Pin Joint (hinge) ----------------------------------
    # The pin is at the end A of the bar = center of block
    pin_x = x2 - half_L * cos(theta2)
    pin_y = y2 - half_L * sin(theta2)
    scatter!(p, [pin_x], [pin_y], color=:red, ms=5,
             markershape=:circle, label="")

    # ----- Draw Bar Center of Mass ---------------------------------
    scatter!(p, [x2], [y2], color=:black, ms=4,
             markershape=:diamond, label="")

    # ----- Draw Free End of Bar ------------------------------------
    free_x = x2 + half_L * cos(theta2)
    free_y = y2 + half_L * sin(theta2)
    scatter!(p, [free_x], [free_y], color=:darkorange, ms=4,
             markershape=:circle, label="")

    return p

end

# ----------------------------------------------------------------
# Mechanism Animation
# ----------------------------------------------------------------

"""
    animate_mechanism(t_steps, solution; filename="results/mechanism.gif", fps=30)

Animate the Spring-Pendulum mechanism over time.

# Arguments
- `t_steps`:  vector of time values
- `solution`: ODE solution object
- `filename`: output GIF file path
- `fps`:      frames per second for the animation
"""
function animate_mechanism(t_steps, solution; filename="results/mechanism.gif", fps=30)

    N = length(t_steps)                              # number of frames

    # pre-compute axis limits from all positions
    all_x1 = [solution(t)[1] for t in t_steps]       # block x-positions
    all_y1 = zeros(N)                                # block y-positions
    all_x2 = [solution(t)[4] for t in t_steps]       # bar center x-positions
    all_y2 = [solution(t)[5] for t in t_steps]       # bar center y-positions

    # compute bar endpoint positions for axis limits
    all_end_x = [solution(t)[4] + half_L * cos(solution(t)[6]) for t in t_steps]
    all_end_y = [solution(t)[5] + half_L * sin(solution(t)[6]) for t in t_steps]
    all_pin_x = [solution(t)[4] - half_L * cos(solution(t)[6]) for t in t_steps]
    all_pin_y = [solution(t)[5] - half_L * sin(solution(t)[6]) for t in t_steps]

    all_xs = vcat(all_x1, all_x2, all_end_x, all_pin_x)
    all_ys = vcat(all_y1, all_y2, all_end_y, all_pin_y)

    pad = 0.08                                                   # padding
    xl  = (minimum(all_xs) - track_len, maximum(all_xs) + pad)   # account for spring/wall
    yl  = (minimum(all_ys) - pad, maximum(all_ys) + pad)

    anim = @animate for i in 1:N

        qi = solution(t_steps[i])[1:6]                # extract coordinates at frame i

        p = plot(
            xlim=xl, ylim=yl,
            aspect_ratio=:equal,
            xlabel="x (m)", ylabel="y (m)",
            title="Spring-Pendulum System  |  t = $(round(t_steps[i], digits=3)) s",
            legend=false,
            size=(700, 500),
            left_margin=5mm, right_margin=5mm,
            bottom_margin=5mm, top_margin=8mm
        )

        # draw the path of the bar's free end up to current frame
        path_x = [solution(t_steps[j])[4] + half_L * cos(solution(t_steps[j])[6]) for j in 1:i]
        path_y = [solution(t_steps[j])[5] + half_L * sin(solution(t_steps[j])[6]) for j in 1:i]
        plot!(p, path_x, path_y, color=:lightgray, lw=1, label="")

        # draw mechanism at current time
        draw_mechanism!(p, qi)

    end

    # save animation as GIF
    gif(anim, filename, fps=fps)
    println("Saved animation: $filename")

end

# ----------------------------------------------------------------
# Static Plot: Positions vs. Time
# ----------------------------------------------------------------

"""
    plot_positions(t_steps, solution; filename="results/positions_vs_time.png")

Plot generalized coordinates vs. time.
"""
function plot_positions(t_steps, solution; filename="results/positions_vs_time.png")

    # extract position data at each time step
    q_all = hcat([solution(t)[1:6] for t in t_steps]...)'    # N×6 matrix, hcat expects multiple arguments (use splat operator), transpose using ' 

    # ----- Panel 1: Block (Body 1) ---------------------------------
    p1 = plot(t_steps, q_all[:, 1], label=L"x_1", lw=2, color=:blue, legend=:topleft) # block x-positions
    plot!(p1, t_steps, q_all[:, 2], label=L"y_1", lw=2, color=:orange)                # block y-positions
    ylabel!(p1, "Position (m)")
    #left_max = maximum(abs.(q_all[:, 1]))                                             # center axis at zero
    #ylims!(p1, (-left_max * 1.2, left_max * 1.2))
    p1r = twinx(p1)
    plot!(p1r, t_steps, q_all[:, 3], label=L"\theta_1", lw=2, ls=:dash, color=:green, legend=:topright) # block orientation
    ylims!(p1r, (-0.5, 0.5))
    ylabel!(p1r, "Angle (rad)")
    title!(p1, "Block Coordinates (Body 1)")

    # ----- Panel 2: Bar (Body 2) -----------------------------------
    p2 = plot(t_steps, q_all[:, 4], label=L"x_2", lw=2, color=:blue, legend=:topleft) # bar x-positions
    plot!(p2, t_steps, q_all[:, 5], label=L"y_2", lw=2, color=:orange)                # bar y-positions
    xlabel!(p2, "Time (s)")
    ylabel!(p2, "Position (m)")
    p2r = twinx(p2)
    plot!(p2r, t_steps, q_all[:, 6], label=L"\theta_2", lw=2, ls=:dash, color=:green, legend=:topright) # bar orientation
    ylabel!(p2r, "Angle (rad)")
    title!(p2, "Bar Coordinates (Body 2)")

    # combine into 2-row layout
    p = plot(p1, p2, layout=(2, 1), size=(800, 700),
             left_margin=10mm, right_margin=15mm,
             bottom_margin=5mm, top_margin=8mm)

    savefig(p, filename)
    println("Saved: $filename")

end

# ----------------------------------------------------------------
# Static Plot: Velocities vs. Time
# ----------------------------------------------------------------

"""
    plot_velocities(t_steps, solution; filename="results/velocities_vs_time.png")

Plot generalized velocities vs. time.
"""
function plot_velocities(t_steps, solution; filename="results/velocities_vs_time.png")

    # extract velocity data at each time step
    dq_all = hcat([solution(t)[7:12] for t in t_steps]...)'    # N×6 matrix

    # ----- Panel 1: Block (Body 1) ------------------------------
    p1 = plot(t_steps, dq_all[:, 1], label=L"\dot{x}_1", lw=2, color=:blue, legend=:topleft)
    plot!(p1, t_steps, dq_all[:, 2], label=L"\dot{y}_1", lw=2, color=:orange)
    ylabel!(p1, "Velocity (m/s)")
    p1r = twinx(p1)
    plot!(p1r, t_steps, dq_all[:, 3], label=L"\dot{\theta}_1", lw=2, ls=:dash, color=:green, legend=:topright)
    ylims!(p1r, (-0.5, 0.5))
    ylabel!(p1r, "Angular Velocity (rad/s)")
    title!(p1, "Block Velocities (Body 1)")

    # ----- Panel 2: Bar (Body 2) --------------------------------
    p2 = plot(t_steps, dq_all[:, 4], label=L"\dot{x}_2", lw=2, color=:blue, legend=:topleft)
    plot!(p2, t_steps, dq_all[:, 5], label=L"\dot{y}_2", lw=2, color=:orange)
    xlabel!(p2, "Time (s)")
    ylabel!(p2, "Velocity (m/s)")
    p2r = twinx(p2)
    plot!(p2r, t_steps, dq_all[:, 6], label=L"\dot{\theta}_2", lw=2, ls=:dash, color=:green, legend=:topright)
    ylabel!(p2r, "Angular Velocity (rad/s)")
    title!(p2, "Bar Velocities (Body 2)")

    p = plot(p1, p2, layout=(2, 1), size=(800, 700),
             left_margin=10mm, right_margin=15mm,
             bottom_margin=5mm, top_margin=8mm)

    savefig(p, filename)
    println("Saved: $filename")

end

# ----------------------------------------------------------------
# Static Plot: Accelerations vs. Time
# ----------------------------------------------------------------

"""
    plot_accelerations(t_steps, ddq_all; filename="results/accelerations_vs_time.png")

Plot generalized accelerations vs. time.
"""
function plot_accelerations(t_steps, ddq_all; filename="results/accelerations_vs_time.png")

    # ----- Panel 1: Block (Body 1) ------------------------------
    p1 = plot(t_steps, ddq_all[:, 1], label=L"\ddot{x}_1", lw=2, color=:blue, legend=:topleft)
    plot!(p1, t_steps, ddq_all[:, 2], label=L"\ddot{y}_1", lw=2, color=:orange)
    ylabel!(p1, "Acceleration (m/s²)")
    p1r = twinx(p1)
    plot!(p1r, t_steps, ddq_all[:, 3], label=L"\ddot{\theta}_1", lw=2, ls=:dash, color=:green, legend=:topright)
    ylims!(p1r, (-0.5, 0.5))
    ylabel!(p1r, "Angular Acceleration (rad/s²)")
    title!(p1, "Block Accelerations (Body 1)")

    # ----- Panel 2: Bar (Body 2) --------------------------------
    p2 = plot(t_steps, ddq_all[:, 4], label=L"\ddot{x}_2", lw=2, color=:blue, legend=:topleft)
    plot!(p2, t_steps, ddq_all[:, 5], label=L"\ddot{y}_2", lw=2, color=:orange)
    xlabel!(p2, "Time (s)")
    ylabel!(p2, "Acceleration (m/s²)")
    p2r = twinx(p2)
    plot!(p2r, t_steps, ddq_all[:, 6], label=L"\ddot{\theta}_2", lw=2, ls=:dash, color=:green, legend=:topright)
    ylabel!(p2r, "Angular Acceleration (rad/s²)")
    title!(p2, "Bar Accelerations (Body 2)")

    p = plot(p1, p2, layout=(2, 1), size=(800, 700),
             left_margin=10mm, right_margin=15mm,
             bottom_margin=5mm, top_margin=8mm)

    savefig(p, filename)
    println("Saved: $filename")

end

# ----------------------------------------------------------------
# Static Plot: Constraint Forces vs. Time
# ----------------------------------------------------------------

"""
    plot_constraint_forces(t_steps, lambda_all; filename="results/constraint_forces.png")

Plot Lagrange multipliers (constraint forces) vs. time.

The multipliers are:
- λ₁: track normal force (y-direction on block)
- λ₂: track moment (torque on block)
- λ₃: pin joint force, x-component
- λ₄: pin joint force, y-component
"""
function plot_constraint_forces(t_steps, lambda_all; filename="results/constraint_forces.png")

    # ----- Panel 1: Track Constraint Forces ---------------------
    p1 = plot(t_steps, lambda_all[:, 1], label=L"\lambda_1" * " (track normal)", lw=2, color=:blue)
    plot!(p1, t_steps, lambda_all[:, 2], label=L"\lambda_2" * " (track moment)", lw=2, color=:orange)
    ylabel!(p1, "Force (N) / Moment (N⋅m)")
    title!(p1, "Track Constraint Forces")
    plot!(p1, legend=:outertopright)

    # ----- Panel 2: Pin Joint Forces ----------------------------
    p2 = plot(t_steps, lambda_all[:, 3], label=L"\lambda_3" * " (pin x-force)", lw=2, color=:blue)
    plot!(p2, t_steps, lambda_all[:, 4], label=L"\lambda_4" * " (pin y-force)", lw=2, color=:orange)
    xlabel!(p2, "Time (s)")
    ylabel!(p2, "Force (N)")
    title!(p2, "Pin Joint Constraint Forces")
    plot!(p2, legend=:outertopright)

    p = plot(p1, p2, layout=(2, 1), size=(800, 700),
             left_margin=10mm, right_margin=15mm,
             bottom_margin=5mm, top_margin=8mm)

    savefig(p, filename)
    println("Saved: $filename")

end

# ----------------------------------------------------------------
# Static Plot: Constraint Residual
# ----------------------------------------------------------------

"""
    plot_constraint_residual(t_steps, solution; filename="results/constraint_residual.png")

Plot the Euclidean norm of the constraint residual vs. time to verify constraint satisfaction.
"""
function plot_constraint_residual(t_steps, solution; filename="results/constraint_residual.png")

    residuals = zeros(length(t_steps))

    for i in eachindex(t_steps)
        qi = solution(t_steps[i])[1:6]
        C_val = C_eqs(qi, t_steps[i])
        residuals[i] = sqrt(sum(C_val .^ 2))     # Euclidean norm/L2 Residual
    end

    residuals = max.(residuals, 1e-16)           # avoid plotting 0 on log scale 

    p = plot(t_steps, residuals, lw=2, color=:red,
             xlabel="Time (s)", ylabel="Constraint L2 Residual",
             title="Constraint L2 Residual vs. Time",
             legend=false, yscale=:log10,
             size=(700, 400),
             left_margin=10mm, right_margin=5mm,
             bottom_margin=5mm, top_margin=8mm)

    savefig(p, filename)
    println("Saved: $filename")

end

# ----------------------------------------------------------------
# Exported Functions/Parameters
# ----------------------------------------------------------------

export draw_mechanism!, animate_mechanism
export plot_positions, plot_velocities, plot_accelerations
export plot_constraint_forces
export plot_constraint_residual

end