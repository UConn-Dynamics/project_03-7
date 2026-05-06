# **ME 5180: Advanced Dynamics | Project 03 | Group 07**

## Group Members:
[Christian DiPietrantonio](mailto:hwp25002@uconn.edu)

## Project Overview
<p align="center">
    <img src="archive/spring_compound-2_bodies.png" width = 500
</p>

In this project, a rigid bar is connected to a sliding block along a
horizontal tracks. The sliding block is connected to a spring that stretches and compresses. The rigid bar $L = 0.4~m$ acts as a compound pendulum.  

1. $x_1-y_1-$ describes block 1 position and orientation, $\theta_1$
2. $x_2-y_2-$ describes the rigid bar position and orientation, $\theta_2$

The applied forces are, 

1. Spring attached to block 1, $F = -k x_1$ where $k = 10~N/m$
2. gravity acting on block 1 and the rigid bar, $F_1 = -m_1g\hat{j}$ and $F_2 = -m_2 g\hat{j}$ where $m_1 = 0.1~kg$ and $m_2 = 0.3~kg$
 
In this project, you need to 

1. determine constraint equations $C(\mathbf{q},~t)$
2. Create an augmented solution method for the dynamic motion of these two moving parts
3. visualize the motion of the system as the two parts complete at least one oscillation
4. calculate and show (graph or vectors) the constraint forces acting on the 2-body system

## Results

TBD

## Conclusions

## Derivations

**Note:** the following derivations use GitHub-compatible Markdown/LaTeX formatting. Certain expressions (e.g., subscripts or matrix notation) may need modification for standard LaTeX/Markdown environments.

## Reproducing Results
From the project directory, run:

```bash
julia main.jl 
```

The program may take a few minutes to run. All final plots are output to the `results/` directory.

## Project Dependencies
This project was developed and tested with **Julia version 1.12.4**

The following packages are required:
- `NonlinearSolve`
- `DifferentialEquations`
- `LinearAlgebra`
- `Plots`
- `Measures`
- `LaTeXStrings`

## Project Structure
```
project_03-7/
|
├───── archive/ # old plots and files
|
├───── notes/   # hand calculations
|
├───── results/ # program outputs (plots)
|
├───── src/     # source Code
|       |
|       ├───── constraints.jl      # constraint equations, Jacobian, mass matrix, forces
|       |
|       ├───── dynamics.jl         # augmented system assembly, ODE function, integration
|       |
|       └───── visualize.jl        # plotting and animation functions
|
├───── main.jl                     # main driver
|
└───── README.md                   # project documentation
```