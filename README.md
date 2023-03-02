# FlipDyn: A game of resource takeovers

This repository contains the implementation of FlipDyn game for scalar and n-dimensional systems. The scalar system takeover policies and value functions are exact. For the n-dimensional system, only an approximate version of the value function and takeover policies are available. The code is implemented in Matlab and provides an interface to solve for scalar and n-dimensional systems and simulate the system dynamics under the takeover policies. 

## Description

 We introduce a game in which two players with opposing objectives seek to repeatedly takeover a common resource. The resource is modeled as a discrete time dynamical system over which a player can gain control after spending a state-dependent amount of energy at each time step. We use a FlipIT-inspired deterministic model that governs which player is in control at every time step. A player’s policy is the probability with which it should spend energy to gain control of the resource at a given time step. Our main results are three-fold. First, we present analytic expressions for the cost-to-go as a function of the hybrid state of the system, i.e., the physical state of the dynamical system and the binary FlipDyn state for any general system with arbitrary costs. These expressions are exact when the physical state is also discrete and has finite cardinality. Second, for a continuous physical state with linear dynamics and quadratic costs, we derive expressions for Nash equilibrium (NE). For scalar physical states, we show that the NE depends only on the parameters of the value function and costs and is independent of the state. Third, we derive an approximate value function for higher dimensional linear systems with quadratic costs. Finally, we illustrate our results through a numerical study on the problem of controlling a linear system in a given environment in the presence of an adversary.

## Gettin Started

Clone or download the repository into your local drive for running the project files.
The following are the inputs to the FlipDyn class.

F - State transition matrix.
B - Control matrix when the defender is in control.
E - Control matrix when the adversary is in control.
K - Linear state feedback gain when the defender is in control.
W - Linear state feedback gain when the adversary is in control.
L - Horizon length of the FlipDyn game. 
Q - State cost.
D - Defender's takeover cost.
A - Adversary's takeover cost.

If you wish to simulate the system once the FlipDyn game is solved, you can provide the following inputs. 

itr - Number of iteration for simulating the system.
x0 - Initial state of the system.

## Main Scripts:

Two main scripts are provided. One for a scalar system and the other for an n-dimensional system. Namely,

''FlipDyn_test_1_dim.m'' - Scalar system script.
''FlipDyn_test_n_dim.m'' - N-dimensional system script.

The FlipDyn system class can be added by using,

''addpath('Flip_DYN')''

## Class files:

''FlipDyn.m'' - FlipDyn class file.
''FlipDyn_LS.m'' - FlipDyn solver for scalar system.
''FlipDyn_NS.m'' - FlipDyn solver for n-dimensional system.
''simulate_sys.m'' - Simulating the system.

# Illustration

![Intro](https://github.com/sandeepbanik/Data-Driven-Resilient-Systems/tree/main/Flip_DYN/Github_readme/Slide1_v2.png)

![Slide 2](https://github.com/sandeepbanik/Data-Driven-Resilient-Systems/tree/main/Flip_DYN/Github_readme/Slide2.PNG)

![Slide 3](https://github.com/sandeepbanik/Data-Driven-Resilient-Systems/tree/main/Flip_DYN/Github_readme/Slide3.PNG)

![Slide 4](https://github.com/sandeepbanik/Data-Driven-Resilient-Systems/tree/main/Flip_DYN/Github_readme/Slide4.PNG)

![Slide 5](https://github.com/sandeepbanik/Data-Driven-Resilient-Systems/tree/main/Flip_DYN/Github_readme/Slide5.PNG)

![Slide 6](https://github.com/sandeepbanik/Data-Driven-Resilient-Systems/tree/main/Flip_DYN/Github_readme/Slide6.PNG)

## Citing

If you find this project useful, we encourage you to 

* Star this repository :star: 
* Cite the [paper](https://ieeexplore.ieee.org/abstract/document/9992387) 
```
@INPROCEEDINGS{9992387,
  author={Banik, Sandeep and Bopardikar, Shaunak D.},
  booktitle={2022 IEEE 61st Conference on Decision and Control (CDC)}, 
  title={FlipDyn: A game of resource takeovers in dynamical systems}, 
  year={2022},
  volume={},
  number={},
  pages={2506-2511},
  doi={10.1109/CDC51059.2022.9992387}}
```

## Acknowledgments

This work was supported by the NSF Award CNS-2134076 under the Secure and Trustworthy Cyberspace (SaTC) program.

