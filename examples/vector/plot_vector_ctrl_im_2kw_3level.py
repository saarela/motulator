"""
2.2-kW induction motor, three-level PWM
=======================================

This example simulates sensorless vector control of a 2.2-kW induction motor
drive with three-level PWM.

"""

# %%
# Imports.

from motulator import model, control
from motulator import BaseValues, plot, plot_extra

# %%
# Compute base values based on the nominal values (just for figures).

base = BaseValues(
    U_nom=400, I_nom=5, f_nom=50, tau_nom=14.6, P_nom=2.2e3, n_p=2)

# %%
# Create the system model.

# Unsaturated machine model, using its inverse-Γ parameters (uncomment to try)
machine = model.im.InductionMachineInvGamma(
   R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, n_p=2)
# Alternatively, configure the machine model using its Γ parameters
# machine = model.im.InductionMachine(
#     R_s=3.7, R_r=2.5, L_ell=.023, L_s=.245, n_p=2)
mechanics = model.Mechanics(J=.015)
converter = model.Inverter(u_dc=540)
mdl = model.im.Drive(machine, mechanics, converter)

# %%
# Configure the control system. You can change compare two- and three-level
# PWM by changing the value of n_levels. The three-level PWM is modeled after
# [#Le2000]_.

# Machine model parameters
par = control.im.ModelPars(
    R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, n_p=2, J=.015)
# Set nominal values and limits for reference generation
ref = control.im.CurrentReferencePars(
    par, i_s_max=1.5*base.i, u_s_nom=base.u, w_s_nom=base.w)
# Create the control system
ctrl = control.im.VectorCtrl(par, ref, T_s=250e-6, sensorless=True, n_levels=3)

# %%
# Set the speed reference and the external load torque.

# Simple acceleration and load torque step
ctrl.w_m_ref = lambda t: (t > .2)*(base.w)
mdl.mechanics.tau_L_t = lambda t: (t > .75)*base.tau_nom

# %%
# Create the simulation object and simulate it. PWM has to be enabled to see the
# results of three-level modulation.

sim = model.Simulation(mdl, ctrl, pwm=True)
sim.simulate(t_stop=1.5)

# %%
# Plot results in per-unit values. By omitting the argument `base` you can plot
# the results in SI units.

plot(sim, base)
plot_extra(sim, t_span=(1.1, 1.14), base=base)

# %%
# .. rubric:: References
#
# .. [#Le2000] Lee, Kim, Hyun, “Carrier based SVPWM method for multi-level system
#    with reduced HDF,” in Conference Record of the 2000 IEEE Industry Applications
#    Conference. Thirty-Fifth IAS Annual Meeting and World Conference on Industrial
#    Applications of Electrical Energy (Cat. No. 00CH37129), 2000,
#    https://doi.org/10.1109/IAS.2000.882151
