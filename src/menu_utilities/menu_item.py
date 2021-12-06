#!usr/bin/env python3
#Python file contain definition for MenuItem
#RKS

# Project imports
from menu_utilities.enum_defs import InstrEnum, StoreRoomEnum
# Python imports

# 3rd party imports

class MenuItem:
    """
    Representation of a menu item the Franka robot can make.
    """
    def __init__(self, name, ingredients, instructions):
        """
        Constructor of the MenuItem class

        Args:
            str name:
                ~name of the menu item
            list ingredients:
                ~list of ingredients needed to make the menu items. The list 
                should contain enum from the StoreRoomEnum
            list instructions:
                ~Recipe of the menu item. The recipe should contain 
                enums from the InstrEnum
        
        Raise:
            RuntimeError:
                Raises a Runtime Error if an ill-defined ingredient or 
                instruction is found.
        """
        self._name = name
        self._ingredients = ingredients
        self._recipe = instructions

        ## Check ingredients
        false_ingredients = []
        for ingredient in self._ingredients:
            if not isinstance(ingredient, StoreRoomEnum):
                false_ingredients.append(ingredient)
        if len(false_ingredients) != 0:
            err_string = f"Unknown ingredients found: {false_ingredients}"
            raise RuntimeError(err_string)


        ## Check instructions
        false_instructions = []
        for instruction in self._recipe:
            if not isinstance(instruction[0], InstrEnum):
                false_instructions.append(instruction)
            if instruction[0] == InstrEnum.GRAB or instruction[0] == InstrEnum.RELEASE:
                if instruction[1] not in self._ingredients:
                    err_string = f"Ingredient {instruction[1]} is missing"
                    raise RuntimeError(err_string)
        if len(false_instructions) != 0:
            err_string = f"Unknown instructions found: {false_instructions}"
            raise RuntimeError(err_string)

    @property
    def name(self):
        """
        Get name of the item
        """
        return self._name

    @name.setter
    def name(self):
        """
        Userbase cannot change the name of the menu item
        """
        print(f"WARN: Menu item {self._name} cannot have it's name changed")
        pass
    
    @property
    def ingredients(self):
        """
        Get the ingredients for the menu item
        """
        return self._ingredients.copy()

    @property
    def recipe(self):
        """
        Get the recipe for the menu item
        """
        return self._recipe.copy()