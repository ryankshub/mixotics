#!usr/bin/env python3
#Python file containing all enum definitions used in project
#RKS

# Project imports

# Python imports
from enum import Enum

# 3rd-party imports


## Instruction Enum
class InstrEnum(Enum):
    """
    Enumeration representing the high-level commands
    to the Franka robot
    
    PREP ~ Prepare a cup for client's drink
    GRAB ~ Grab a certain ingredient
    POUR ~ Pour grabbed ingredient
    MIX ~ Mix current ingredients in client's cup
    DELIVER ~ Give cup to client
    """
    PREP = 0
    GRAB = 1
    POUR = 2
    MIX = 3
    DELIVER = 4

## StoreRoom Enum
class StoreRoomEnum(Enum):
    """
    Enumeration representing the current ingredients we support
    """
    WATER = 0
    LEMON = 1
    TEA = 2
    