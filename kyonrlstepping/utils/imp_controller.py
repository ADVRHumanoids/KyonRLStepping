# wrapper on top of ArticulationController to allow full joint impedance control 
# while also modulating the control frequency wrt the simulation frequency 
# (we only allow for integer multiples of the current simulation frequency)

# internally, the control mode of the whole articulation must be switched to "effort" 

# we use torch as a backend and configure optionally also the device on which to put the data

# we get references from ArticulationController

# and wrap setter methods for both impedance and references from ArticulationController