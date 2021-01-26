# KSP Launch Automation Project

## Introduction

### Kerbal Space Program

[Kerbal Space Program](https://en.wikipedia.org/wiki/Kerbal_Space_Program) is a popular space flight simulation game that features a realistic orbital physics engine and is a game I grew up playing. Performing launches, orbit circularization, and Hohmann transfers manually was fun and all but after learning coding, I wanted to try my hand at automating this process.

### Kerbal Operating System

[Kerbal Operating System](https://ksp-kos.github.io/KOS/index.html) (kOS) is a mod for Kerbal Space Program that introduces a simulated computer capable of running programs written in a scripting language called **kerboscript** (.ks).  With this, I was able to create a script that successfully launched my spacecraft to orbit.

## Implementation

### launch.ks

The process from getting a payload from the launch pad to a circular orbit around Kerbin is broken into three separate phases:

 - Phase 1: Flying from the launch pad and exiting the atmosphere.
 - Phase 2: Performing corrective burns to get the apoapsis of the sub-orbital trajectory to the target circular orbit altitude.
 - Phase 3: Performing a burn to circularize the orbit at the sub-orbital trajectory's apoapsis to insert the spacecraft into a circular orbit.

A variety of different functions were created to meet the demands of each phase and are listed below.

#### Phase 1: Launch

The tasks carried out during Phase 1 in guiding and controlling the rocket from the rocket into a suborbital trajectory are as follows:

 - `throttleFunction(vesVelocity, vesAltitude)`: Controlling the throttle to maintain the airspeed at or a below a target airspeed, which was a function of the density of the atmosphere at the current altitude so as to reduce energy lost due to drag.
	 - The target airspeed was defined by an exponential function that was the inverse of [the density versus altitude curve of Kerbin](https://wiki.kerbalspaceprogram.com/wiki/File:Kerbin_atmospheric_density.png). 
 - `pitchFunction(vesAltitude)`: Maintaining the pitch of the rocket following a specified piecewise function to ensure a successful gravity turn. 
 - `stagingFunction()`: Staging the rocket when the currrent stage runs out of fuel.

#### Phase 2: Altitude Correction Burns

After exiting the atmosphere of Kerbin, an additional corrective burn is performed to place the apoapsis of the trajectory at the proper target orbit altitude. The following function was used to carry this out:
- `correctionBurn(targetOrbitAltitude, currentApoapsis)`: Implements a proportional controller with saturation between `targetOrbitAltitude` and `currentApoapsis` to throttle the engine until the two values are the same.

#### Phase 3: Orbit Circularization

A maneuver node is created at the apoapsis to perform a circularization burn. The time required to execute the burn is calculated and the burn is timed to begin before the maneuver node is reached by a calculated burn centroid time. The burn centroid time is calculated to be the time into a burn at which the <img src="https://render.githubusercontent.com/render/math?math=\Delta%20v"> achieved between the start of the burn and the burn centroid time and the <img src="https://render.githubusercontent.com/render/math?math=\Delta%20v"> achieved between the burn centroid time and the end of the burn are equal. This is done to achieve an orbit as close as possible to the one predicted by the maneuver node, which represents an orbit achieved using an instantaneous velocity change.

- `circDeltaV()`: Returns the <img src="https://render.githubusercontent.com/render/math?math=\Delta%20v"> required to perform an orbit circularization for use in creating the maneuver node. This is done by calculating the velocity required for a circular orbit with an altitude equal to the altitude of the current apoapsis, <img src="https://render.githubusercontent.com/render/math?math=v_c">, calculating the velocity the vessel will have when it reaches the apoapsis of its orbit, <img src="https://render.githubusercontent.com/render/math?math=v_a">, and taking the difference between the two. 

![velocity in circular orbit](https://latex.codecogs.com/gif.latex?%5CLARGE%20v_c%20%3D%20%5Csqrt%7B%5Cfrac%7BGM%7D%7B%5Cleft%28%20R_E&plus;h_a%5Cright%20%29%5E2%7D%7D)

![velocity at apoapsis of suborbital trajectory](https://latex.codecogs.com/gif.latex?%5CLARGE%20v_a%20%3D%20%5Csqrt%7BGM%5Cleft%28%20%5Cfrac%7B2%7D%7BR_E&plus;h_a%7D%20-%20%5Cfrac%7B1%7D%7Ba%7D%20%5Cright%20%29%7D)

![semimajor axis](https://latex.codecogs.com/gif.latex?%5CLARGE%20a%20%3D%20%5Cfrac%7Br_a%20&plus;%20r_p%7D%7B2%7D)

![delta v required](https://latex.codecogs.com/gif.latex?%5CLARGE%20%5CDelta%20v%20%3D%20v_c%20-%20v_a)

- `maneuverNodeThrottler()`: Implements a proportional controller with saturation between the vessel's current velocity and target velocity for a circular orbit to throttle the engine until those two values are the same.
- `executeNextManeuverNode()`:  
	- Turns the spacecraft in the direction to execute the next maneuver node.
	- Checks the <img src="https://render.githubusercontent.com/render/math?math=\Delta%20v"> present in the current stage and compares it to the <img src="https://render.githubusercontent.com/render/math?math=\Delta%20v"> required to execute the maneuver node.
		- If the <img src="https://render.githubusercontent.com/render/math?math=\Delta%20v"> of the current stage will be enough to successfully execute the maneuver node, calculate the burn centroid time and time the start of the burn such that it begins before the time of the maneuver node by the burn centroid time. The burn centroid time is calculated using the following equation.
![Single stage centroid equation](https://latex.codecogs.com/gif.latex?%5Chuge%20t_c%20%3D%20%5Cfrac%7Bm_i%20g_0%20I_%5Ctext%7Bsp%7D%7D%7BF_%5Ctext%7Bthrust%7D%7D%20%5Cleft%28%201%20-%20e%5E%7B-%5Cfrac%7B1%7D%7B2%7D%5Cln%5Cleft%28%20%5Cfrac%7Bm_i%7D%7Bm_f%7D%20%5Cright%20%29%7D%20%5Cright%20%29)
		- If the <img src="https://render.githubusercontent.com/render/math?math=\Delta%20v"> of the current stage is not enough to successfully carry out the maneuver node, assume that the burn will occur over two stages.  Approximate the net burn centroid time across both stages by calculating the burn centroid time of the current stage and of the following stage before getting the <img src="https://render.githubusercontent.com/render/math?math=\Delta%20v">-weighted average as follows.
		![current stage burn time centroid](https://latex.codecogs.com/gif.latex?%5Chuge%20t_1_c%20%3D%20%5Cfrac%7Bm_1_i%20g_0%20I_%7B1%5Ctext%7Bsp%7D%7D%7D%7BF_%7B1%5Ctext%7Bthrust%7D%7D%7D%20%5Cleft%28%201%20-%20e%5E%7B-%5Cfrac%7B1%7D%7B2%7D%5Cln%5Cleft%28%20%5Cfrac%7Bm_1_i%7D%7Bm_1_f%7D%20%5Cright%20%29%7D%20%5Cright%20%29)
![second stage burn time centroid](https://latex.codecogs.com/gif.latex?%5Chuge%20t_%7B2c%7D%20%3D%20t_%7B1b%7D%20&plus;%20%5Cfrac%7Bm_%7B2i%7Dg_0I_%7B2%5Ctext%7Bsp%7D%7D%7D%7BF_%7B2%5Ctext%7Bthrust%7D%7D%7D%5Cleft%28%201%20-%20e%5E%7B-%5Cfrac%7B1%7D%7B2%7D%20%5Cln%20%5Cleft%28%5Cfrac%7Bm_%7B2i%7D%7D%7Bm_%7B2f%7D%7D%20%5Cright%20%29%7D%20%5Cright%20%29)
![net burn time centroid](https://latex.codecogs.com/gif.latex?%5Chuge%20t_%7Bc%7D%20%3D%20%5Cfrac%7B%5CDelta%20v_1%20t_%7B1c%7D%20&plus;%20%5CDelta%20v_2%20t_%7B2c%7D%7D%7B%5CDelta%20v_1%20&plus;%20%5CDelta%20v_2%7D)



## Skills Demonstrated

 - Demonstrated strong understanding of orbital mechanics and propulsions via implementation of a spacecraft orbit circularization algorithm.
	 - Guided spacecraft through the use of proportional controllers to manage velocity.
	 - Proved understanding of <img src="https://render.githubusercontent.com/render/math?math=\Delta%20v"> and how to change mission procedures based on budgeted <img src="https://render.githubusercontent.com/render/math?math=\Delta%20v">.
 - Demonstrated ability to provide clear documentation of code with explanations and equations.
 - Demonstrated ability to quickly learn new coding languages and apply computer science principles including loops, conditionals, and functions.
