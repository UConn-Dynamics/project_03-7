### A Pluto.jl notebook ###
# v0.20.21

using Markdown
using InteractiveUtils

# ╔═╡ f17103ea-06bf-11f1-a2b0-79e68ed152eb
md"""# Project_03 - Multibody Dynamic modeling

![Sliding compound pendulum with a support block connected to a spring and rotating compound pendulum](https://raw.githubusercontent.com/cooperrc/me5180-project_02/refs/heads/main/spring_compound-2_bodies.png)

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
"""

# ╔═╡ 0d9be664-d7c5-4084-add2-25e5418742d6


# ╔═╡ 00000000-0000-0000-0000-000000000001
PLUTO_PROJECT_TOML_CONTENTS = """
[deps]
"""

# ╔═╡ 00000000-0000-0000-0000-000000000002
PLUTO_MANIFEST_TOML_CONTENTS = """
# This file is machine-generated - editing it directly is not advised

julia_version = "1.12.4"
manifest_format = "2.0"
project_hash = "71853c6197a6a7f222db0f1978c7cb232b87c5ee"

[deps]
"""

# ╔═╡ Cell order:
# ╟─f17103ea-06bf-11f1-a2b0-79e68ed152eb
# ╠═0d9be664-d7c5-4084-add2-25e5418742d6
# ╟─00000000-0000-0000-0000-000000000001
# ╟─00000000-0000-0000-0000-000000000002
