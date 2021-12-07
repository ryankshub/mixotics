#!/usr/bin/env python3
# Test file for menu_parser in menu_utilities
# RKS

# Project imports
from menu_utilities.enum_defs import InstrEnum, StoreRoomEnum
from menu_utilities.menu_parser import parse_menu_xml

# Python imports
import unittest

# 3rd party imports

# Constants
PKG = 'final_project_mixotics'

class MenuParserTestCase(unittest.TestCase):
    """
    Test cases for menu parser
    """
    def test_basic_menu(self):
        """
        Given a simple menu, ensure all drink, ingredients, and
        recipes are parsed correctly
        """
        # Basic menu makes water, lemonade, and tea
        menu_file = "test_artifacts/basic_menu.xml"
        menu_dict = parse_menu_xml(menu_file)

        #Grab and check each menu item in dict
        item_names = ["water", "lemonade", "icedtea"]
        item_found = [False, False, False]
        for key in menu_dict.keys():
            # Ensure no mystery item made it on the menu
            self.assertTrue(key in item_names, 
                f"Menu Item {key} should not be in dictionary")
            item_found[item_names.index(key)] = True
        # Ensure all expected drinks made it on the menu
        self.assertTrue(all(item_found), "Missing expected menu item(s)")

        # Test Water item
        Water = menu_dict["water"]
        proper_ings_water = [StoreRoomEnum.WATER]
        proper_rep_water = [(InstrEnum.GRAB, StoreRoomEnum.WATER),
                            (InstrEnum.POUR, 20),
                            (InstrEnum.RELEASE, StoreRoomEnum.WATER),
                            (InstrEnum.DELIVER, None)]
        #Check name, ingredient, and recipe
        self.assertEqual("water", Water.name, 
            f"Water item name should be water, not {Water.name}")
        self.assertEqual(proper_ings_water, Water.ingredients,
            f"Ingredients {[x.name for x in Water.ingredients]} are not correct")
        self.assertEqual(proper_rep_water, Water.recipe, 
            f"Recipe {[(x[0].name, x[1]) for x in Water.recipe]} is not correct")

        # Test Lemonade item
        Lemonade = menu_dict["lemonade"]
        proper_ings_lemon = [StoreRoomEnum.WATER, StoreRoomEnum.LEMON]
        proper_rep_lemon = [(InstrEnum.GRAB, StoreRoomEnum.WATER),
                            (InstrEnum.POUR, 10),
                            (InstrEnum.RELEASE, StoreRoomEnum.WATER),
                            (InstrEnum.GRAB, StoreRoomEnum.LEMON),
                            (InstrEnum.POUR, 10),
                            (InstrEnum.RELEASE, StoreRoomEnum.LEMON),
                            (InstrEnum.DELIVER, None)]
        #Check name, ingredient, and recipe
        self.assertEqual("lemonade", Lemonade.name, 
            f"Lemonade item name should be lemonade, not {Lemonade.name}")
        self.assertEqual(proper_ings_lemon, Lemonade.ingredients,
            f"Ingredients for Lemonade are not correct: {Lemonade.ingredients}")
        self.assertEqual(proper_rep_lemon, Lemonade.recipe, 
            f"Recipe for Lemonade is not correct: {Lemonade.recipe}")

        # Test Tea item
        Tea = menu_dict["icedtea"]
        proper_ings_tea = [StoreRoomEnum.WATER, StoreRoomEnum.TEA]
        proper_rep_tea = [(InstrEnum.GRAB, StoreRoomEnum.WATER),
                          (InstrEnum.POUR, 10),
                          (InstrEnum.RELEASE, StoreRoomEnum.WATER),
                          (InstrEnum.GRAB, StoreRoomEnum.TEA),
                          (InstrEnum.POUR, 10),
                          (InstrEnum.RELEASE, StoreRoomEnum.TEA),
                          (InstrEnum.DELIVER, None)]
        #Check name, ingredient, and recipe
        self.assertEqual("icedtea", Tea.name, 
            f"Tea item name should be icedtea, not {Tea.name}")
        self.assertEqual(proper_ings_tea, Tea.ingredients,
            f"Ingredients for Tea are not correct: {Tea.ingredients}")
        self.assertEqual(proper_rep_tea, Tea.recipe, 
            f"Recipe for Tea is not correct {Tea.recipe}")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'menu_parser_test', MenuParserTestCase)

        