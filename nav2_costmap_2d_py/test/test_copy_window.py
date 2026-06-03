# Copyright (c) 2026 Alberto J. Tudela Roldán
# Copyright (c) 2026 Grupo Avispa, DTE, Universidad de Málaga
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Unit tests for Costmap2D.copy_window.

Python port of nav2_costmap_2d/test/unit/copy_window_test.cpp.
"""

import unittest

from nav2_costmap_2d_py.core.costmap_2d import Costmap2D


class TestCopyWindow(unittest.TestCase):
    """Test suite for Costmap2D.copy_window."""

    def test_copy_valid_window(self) -> None:
        """Mirror copy_window_test.cpp CopyWindow.copyValidWindow."""
        src = Costmap2D(10, 10, 0.1, 0.0, 0.0)
        dst = Costmap2D(5, 5, 0.2, 100.0, 100.0)
        # Adding 2 marked cells to source costmap
        src.set_cost(2, 2, 100)
        src.set_cost(5, 5, 200)

        self.assertTrue(dst.copy_window(src, 2, 2, 6, 6, 0, 0))
        # Check that both marked cells were copied to destination costmap
        self.assertEqual(dst.get_cost(0, 0), 100)
        self.assertEqual(dst.get_cost(3, 3), 200)

    def test_copy_invalid_window(self) -> None:
        """Mirror copy_window_test.cpp CopyWindow.copyInvalidWindow."""
        src = Costmap2D(10, 10, 0.1, 0.0, 0.0)
        dst = Costmap2D(5, 5, 0.2, 100.0, 100.0)

        # Case 1: incorrect source bounds
        self.assertFalse(dst.copy_window(src, 9, 9, 11, 11, 0, 0))
        # Case 2: incorrect destination bounds
        self.assertFalse(dst.copy_window(src, 0, 0, 1, 1, 5, 5))
        self.assertFalse(dst.copy_window(src, 0, 0, 6, 6, 0, 0))


if __name__ == '__main__':
    unittest.main()
