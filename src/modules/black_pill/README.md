
# What is this crate?
This is a crate whose objective is to create even another 
abstraction layer for our specific microcontroller (RobotDyn Black pill).
This is less efficient and not recommended in industrial application but 
is makes development easier for people not familiarized with microcontrollers,
hiding all the complexity from the user and passing it to the compiler and the 
library in arduino style. 

# Why?
This crate was created to help with the development at Club Robot de L'INSA Toulouse.
This enables all students to help with the development.

# Crate logic
This crate is centered around the BlackPill struct. It creates a common type for all 
the configured GPIOs, the ```Channel``` type. We can use a channel for read or write
to adn from the GPIO. This channel has a mode in which it is configured and the code will
panic if we try to use it in another mode before configuring it.

We also have the mode enum and the pin enum, those are only used for configuration 
purposes, only to pass the information for in which mode we want to configure
a channel.

# IMPORTANT
We didn't thing about remap in this crate! It is supposed to be simple and check for the basic uses at compile time not more!



