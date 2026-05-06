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

end

# ------------------------------------------------------------------------
# Main Driver Call
# ------------------------------------------------------------------------

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end