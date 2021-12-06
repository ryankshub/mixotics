#!/usr/bin/env python3

import unittest
import rospy
from final_project_mixotics.msg import CupDetection

class CupDetectionTest(unittest.TestCase):
    """Tests cup detection publisher in Vision node."""
    def __init__(self, *args):
        super(CupDetectionTest, self).__init__(*args)
        rospy.init_node('cup_detection_test')
        self.msg = rospy.wait_for_message('cup_status', CupDetection)

    def test_array(self):
        """Tests truth values in published cup_status array."""
        array = CupDetection([True, True, False, True])
        self.assertEqual(array, self.msg)

if __name__ == "__main__":
    import rostest
    rostest.rosrun('final_project_mixotics', 'cup_detection_test', CupDetectionTest)
