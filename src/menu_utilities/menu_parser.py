#!usr/bin/env python3
#Python file for various helper functions for processing menus
#RKS

# Project imports
from menu_utilities.enum_defs import InstrEnum, StoreRoomEnum
from menu_utilities.menu_item import MenuItem

# Python imports
#NOTE: This should only be used with known menus 
#(not protected from malicious-formed data)
import xml.etree.ElementTree as ET 

# 3rd-party imports

###
def parse_menu_xml(menu_file):
    """
    Given the filepath of a menu xml file, this function parses
    the file and returns a dictionary of menu string-> menu items

    Args:
        Str menu_file:
            -Filepath of menu configuration xml
    Rtn:
        dict menu_dict:
            -Mapping from the name of the menu item to a MenuItem
    """
    rtn_dict = {}
    tree = ET.parse(menu_file)
    menu = tree.getroot()
    for drink in menu:
        # Get Drink Name
        name_node = drink.find('name')
        name = name_node.text

        # Get Drink Ingredients
        ings_node = drink.find('ingredients')
        ings_list = []
        for ing_node in ings_node:
            ing_enum = StoreRoomEnum[ing_node.text.upper()]
            ings_list.append(ing_enum)

        # Get Recipe
        recipe_node = drink.find('recipe')
        recipe_instrs = []
        for instr_node in recipe_node:
            instr_enum = InstrEnum[instr_node.text.upper()]
            if instr_enum == InstrEnum.GRAB or instr_enum == InstrEnum.RELEASE:
                instr_item = instr_node.attrib['item']
                recipe_instrs.append((instr_enum, StoreRoomEnum[instr_item.upper()]))
            elif instr_enum == InstrEnum.POUR:
                instr_item = int(instr_node.attrib['amount'])
                recipe_instrs.append((instr_enum, instr_item))
            else:
                recipe_instrs.append((instr_enum, None))
                
        # Add drink to dictionary
        drink_item = MenuItem(name, ings_list, recipe_instrs)
        rtn_dict[name] = drink_item

    return rtn_dict
