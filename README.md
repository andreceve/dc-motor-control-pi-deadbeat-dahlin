# DC Motor Identification and Digital Control on TI C2000

Laboratory project for the course **Laboratory of Automation Systems** (M.Sc. in Automation Engineering, University of Bologna, 2024).

A brushed DC motor coupled to a brushless machine (used as load) is identified and controlled in real time with a **Texas Instruments TMS320F28379D** LaunchPad. All the firmware is generated from **MATLAB/Simulink** (Embedded Coder, C2000 support package) and a second Simulink model running on the PC talks to the board over serial to send references and log signals.

The project covers the full workflow, from sensing to controller design:

1. **Sensing and actuation**: PWM generation, current measurement through a shunt resistor and ADC (with automatic offset calibration), speed measurement from a quadrature encoder with a discrete low-pass filter.
2. **Parameter identification** of the electrical (R, L) and mechanical (J, b) dynamics with least squares on a first-order ARX model.
3. **Cascade PI control** (inner current loop, outer speed loop) tuned by pole-zero cancellation, with saturation, anti-windup and speed feedforward.
4. **Direct digital design**: **Deadbeat** and **Dahlin** speed controllers, with a polynomial anti-windup scheme, tested in simulation and on the real motor.

<p align="center">
  <img src="images/setup/setup_photo.png" width="320" alt="Test bench">
</p>

## Hardware and software

| Item | Details |
|---|---|
| MCU | TI TMS320F28379D (C2000), 200 MHz |
| Motor | Brushed DC, 24 V, 1500 rpm nominal, 0.22 Nm, km = 0.12 Nm/A |
| Load | Brushless machine on the same shaft (also used with a resistive load as disturbance) |
| Sensors | Quadrature encoder 2048 ppr (x4 decoding), shunt resistor 7 mΩ + gain 10 amplifier, 12-bit ADC |
| Sampling / PWM | 10 kHz (Ts = 0.1 ms) |
| Software | MATLAB/Simulink, Embedded Coder, C2000 Microcontroller Blockset, serial communication with the host PC |

## Repository structure

```
├── 01_cascade_pi_control/      Cascade PI (current + speed) tuned with datasheet parameters
├── 02_parameter_estimation/    Acquisition model, least-squares identification of R, L, J, b,
│                               cascade PI retuned with the estimated parameters
├── 03_deadbeat_control/        Deadbeat speed controller (simulation + hardware models)
├── 04_dahlin_control/          Dahlin speed controller (simulation + hardware models)
├── data/                       Experimental and simulated acquisitions (.mat) + plotting script
└── images/
    ├── setup/                  Test bench
    ├── models/                 Screenshots of the Simulink models
    ├── code/                   Screenshots of the main MATLAB functions
    └── results/                Experimental and simulation results
```

Each control folder contains:

* `BaseModelForControl_Parameters_*.m`: hardware, sensing and controller parameters (run it first)
* `BasePerTracing_*.slx`: model deployed on the C2000 board
* `CommandInterface_*.slx`: host-side model (references, data logging via serial)
* `*Model.slx` + `*Model_Parameters.m` (deadbeat and Dahlin only): simulation model and controller design

## 1. Sensing and firmware structure

The board model is split into an analog reading area (motor current, DC-link voltage, motor voltage), an encoder area (position to speed, filtered) and a control area that computes the requested voltage for the PWM module.

<p align="center">
  <img src="images/models/hardware_model_toplevel.png" width="600" alt="Board model">
</p>

The current offset of the sensing chain is estimated at start-up with a slow low-pass filter, and the speed obtained from the encoder counts is filtered with a first-order discrete low-pass filter (backward Euler, τ = 1 ms).

## 2. Parameter identification

The motor is excited with a square-wave voltage (electrical part, rotor practically still) and a square-wave current/torque reference (mechanical part). Each subsystem is a first-order system, so after discretization

$$y(k) = \alpha_1\, y(k-1) + \alpha_2\, u(k)$$

and the parameters are obtained by least squares, $\alpha = (\Phi^T\Phi)^{-1}\Phi^T y$:

* electrical part, $I(s)/V(s) = 1/(Ls + R)$: $\;L = \alpha_1 T_s/\alpha_2$, $\;R = (1-\alpha_1)/\alpha_2$
* mechanical part, $\Omega(s)/T(s) = 1/(Js + b)$: $\;J = \alpha_1 T_s/\alpha_2$, $\;b = (1-\alpha_1)/\alpha_2$

| Parameter | Datasheet | Estimated |
|---|---|---|
| Armature resistance R | 3.2 Ω | 3.337 Ω |
| Armature inductance L | 4.1 mH | 6.1 mH |
| Total inertia J (DC + brushless rotor) | 7.32·10⁻⁵ kg·m² | 8.36·10⁻⁵ kg·m² |
| Viscous friction b | 8.6·10⁻⁴ N·m·s (from τ_mech) | 8.58·10⁻⁴ N·m·s |

The identified models reproduce the measured current and speed much better than the datasheet ones (top: datasheet, bottom: estimated):

<p align="center">
  <img src="images/results/LR_ESTIM.png" width="420" alt="R-L estimation">
  <img src="images/results/JB_ESTIM.png" width="420" alt="J-b estimation">
</p>

To reproduce the estimation, open `02_parameter_estimation/` and run `LR_estimation_DCMotor.m` or `Jb_estimation_DCMotor.m`: they load the acquisitions from `data/`, estimate the parameters and compare the two models in simulation.

## 3. Cascade PI control

Both loops use a discrete PI (Tustin) with output saturation and anti-windup (integrator clamping), enabled from the host interface. The controllers are tuned by pole-zero cancellation:

* current loop: $T_{i,I} = L/R$, $K_{p,I} = L/\tau_i$ with $\tau_i = 1$ ms
* speed loop: $T_{i,\omega} = J/b$, $K_{p,\omega} = J/(\tau_\omega k_m)$ with $\tau_\omega = 10\,\tau_i$

A speed feedforward term can also be added. The flag `useEstimatedParams` in `BaseModelForControl_Parameters_*.m` selects datasheet or estimated parameters for the tuning.

<p align="center">
  <img src="images/models/cascade_pi_controller.png" width="650" alt="Cascade PI">
</p>

<p align="center">
  <img src="images/results/CASCADEPI_Current_confronto4.png" width="420" alt="Current loop comparison">
  <img src="images/results/CASCADEPI_Speed_confronto_sat2.png" width="420" alt="Speed loop comparison">
</p>

Further tests: staircase speed reference, trajectory tracking with and without feedforward, and disturbance rejection when a resistive load is suddenly connected to the brushless machine.

<p align="center">
  <img src="images/results/StairReference2.png" width="280" alt="Stair reference">
  <img src="images/results/WithFF.png" width="280" alt="With feedforward">
  <img src="images/results/ResistiveLoadSpeed.png" width="280" alt="Resistive load">
</p>

## 4. Deadbeat and Dahlin controllers

The plant used for the direct design is the full DC motor model from voltage to speed, built with the estimated parameters and discretized with ZOH at Ts = 0.1 ms:

$$G_m(s) = \frac{k_m}{LJ s^2 + (RJ + bL)s + k_m^2 + bR}$$

* **Deadbeat**: $D(z) = \dfrac{1}{G_p(z)}\,\dfrac{z^{-1}}{1-z^{-1}}$, the closed loop is a pure one-step delay.
* **Dahlin**: the closed loop is a first-order system with time constant λ = 20 ms and delay θ = Ts,
  $G_m(z) = \dfrac{(1-e^{-T_s/\lambda})\,z^{-N-1}}{1-e^{-T_s/\lambda}z^{-1}}$, and $D(z) = \dfrac{1}{G_p(z)}\dfrac{G_m(z)}{1-G_m(z)}$.

Writing $D(z) = B(z)/A(z)$, the controller is implemented with a polynomial anti-windup structure, $u = \frac{B}{F}e + \frac{F-A}{F}u_{sat}$, where F(z) is a stable polynomial chosen by design. This keeps the controller internal state consistent with the saturated control action.

<p align="center">
  <img src="images/models/polynomial_controller_antiwindup_scheme.png" width="550" alt="Anti-windup scheme">
</p>

Each controller was tested in four configurations: ideal (no saturation), with saturation and no anti-windup, with saturation and anti-windup, in simulation and on the real motor.

| | Without anti-windup | With anti-windup |
|---|---|---|
| Deadbeat | <img src="images/results/DEADBEAT_SpeedNOAW5.png" width="380"> | <img src="images/results/DEADBEAT_SpeedSIAW2.png" width="380"> |
| Dahlin | <img src="images/results/DAHLIN_SpeedNOAW5.png" width="380"> | <img src="images/results/DAHLIN_SpeedSIAW2.png" width="380"> |

Without anti-windup the saturated real system oscillates around the reference, while with the anti-windup structure the response follows the ideal one closely. The control action plots in `images/results/CONTROLACTION_*` show the ringing typical of deadbeat control (cancellation of a plant zero close to z = -1), which is much smaller with Dahlin's design.

## How to run

1. Install MATLAB/Simulink with Embedded Coder and the *C2000 Microcontroller Blockset*.
2. Open the folder of the controller to test and run `BaseModelForControl_Parameters_*.m`; for deadbeat/Dahlin run `*Model_Parameters.m` afterwards.
3. Simulation only: open `*Model.slx` (or `LR_Model.slx` / `Jb_Model.slx`) and run it.
4. Hardware: build and deploy `BasePerTracing_*.slx` on the LaunchPad, then run `CommandInterface_*.slx` on the PC (set `comport` in the parameter script to the board's COM port).
5. To compare logged acquisitions, edit and run `data/plot_acquisitions.m`.

## Author

Andrea Cevenini · [GitHub](https://github.com/andreceve) · [LinkedIn](https://linkedin.com/in/andrea-cevenini-499754237)
