#!/usr/bin/env python3
# Test file for menu_item in menu_utilities
# RKS

# Project Imports
from menu_utilities.enum_defs import InstrEnum, StoreRoomEnum
import menu_utilities.menu_item as mitem

# Python Imports
import unittest

# 3rd-party Imports

# Constants
PKG = 'final_project_mixotics'

class MitemTestCase(unittest.TestCase):
    """
    Test Case for menu item object
    """
    def test_basic_case(self):
        """
        Basic test case of defining 'lemonade'
        """
        good_name = "lemonade"
        good_ingreds = [StoreRoomEnum['WATER'], StoreRoomEnum['LEMON']]
        good_recipe = [(InstrEnum['PREP'], None), 
                       (InstrEnum['GRAB'], StoreRoomEnum['WATER']),
                       (InstrEnum['POUR'], None),
                       (InstrEnum['GRAB'], StoreRoomEnum['LEMON']),
                       (InstrEnum['POUR'], None),
                       (InstrEnum['MIX'], None),
                       (InstrEnum['DELIVER'], None)]
        good_mitem = mitem.MenuItem(good_name, good_ingreds, good_recipe)

        self.assertEqual(good_mitem.name, good_name, 
                        f"Name of Menu Item should be {good_name}")
        self.assertEqual(good_mitem.ingredients, good_ingreds,
                        f"Ingredients for Item should be {good_ingreds}")
        self.assertEqual(good_mitem.recipe, good_recipe,
                        f"Recipe for Item should be {good_recipe}")


    def test_nominal_case(self):
        """
        Test the case where not all ingredients are used
        This is a valid menu item
        """
        good_name = "special"
        good_ingreds = [StoreRoomEnum['WATER'], StoreRoomEnum['LEMON']]
        good_recipe = [(InstrEnum['PREP'], None), 
                       (InstrEnum['GRAB'], StoreRoomEnum['WATER']),
                       (InstrEnum['POUR'], None),
                       (InstrEnum['MIX'], None),
                       (InstrEnum['DELIVER'], None)]
        good_mitem = mitem.MenuItem(good_name, good_ingreds, good_recipe)

        self.assertEqual(good_mitem.name, good_name, 
                        f"Name of Menu Item should be {good_name}")
        self.assertEqual(good_mitem.ingredients, good_ingreds,
                        f"Ingredients for Item should be {good_ingreds}")
        self.assertEqual(good_mitem.recipe, good_recipe,
                        f"Ingredients for Item should be {good_ingreds}")


    def test_invalid_ingredient(self):
        """
        Test the case where an ill-formatted ingredient has been passed in
        This is an invalid menu item
        """
        bad_name = "badstore"
        bad_ingreds = [StoreRoomEnum['WATER'], "Lemon"]
        bad_recipe = [(InstrEnum['PREP'], None), 
                       (InstrEnum['GRAB'], StoreRoomEnum['WATER']),
                       (InstrEnum['POUR'], None),
                       (InstrEnum['MIX'], None),
                       (InstrEnum['DELIVER'], None)]
        caught_error = False
        try:
            bad_mitem = mitem.MenuItem(bad_name, bad_ingreds, bad_recipe)
        except RuntimeError:
            caught_error = True

        self.assertTrue(caught_error, 
                        "Item should have throw exception due to bad ingredients")


    def test_missing_ingredient(self):
        """
        Test the case where a recipe contains an ingredient 
        that is not listed
        This is an invalid menu item
        """
        bad_name = "needslemon"
        bad_ingreds = [StoreRoomEnum['WATER']]
        bad_recipe = [(InstrEnum['PREP'], None), 
                       (InstrEnum['GRAB'], StoreRoomEnum['LEMON']),
                       (InstrEnum['POUR'], None),
                       (InstrEnum['MIX'], None),
                       (InstrEnum['DELIVER'], None)]
        caught_error = False
        try:
            bad_mitem = mitem.MenuItem(bad_name, bad_ingreds, bad_recipe)
        except RuntimeError:
            caught_error = True
        
        self.assertTrue(caught_error, 
                        "Item should have throw exception due to missing ingredients")


    def test_invalid_recipe(self):
        """
        Test an ill-formatted recipe
        This is an invalid menu item
        """
        bad_name = "recipeblog"
        bad_ingreds = [StoreRoomEnum['WATER'], StoreRoomEnum['LEMON']]
        bad_recipe = [(InstrEnum['PREP'], None), 
                       ("SHAKE THE JUG", StoreRoomEnum['WATER']),
                       (InstrEnum['POUR'], None),
                       (InstrEnum['MIX'], None),
                       (InstrEnum['DELIVER'], None)]
        caught_error = False
        try:
            bad_mitem = mitem.MenuItem(bad_name, bad_ingreds, bad_recipe)
        except RuntimeError:
            caught_error = True
        
        self.assertTrue(caught_error,  
                        "Item should have throw exception due to bad recipe")
    
            

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'menu_item_test', MitemTestCase)