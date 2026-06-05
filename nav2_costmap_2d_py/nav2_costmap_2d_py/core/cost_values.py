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
Cost value constants for the nav2_costmap_2d Python port.

It mirrors the nav2_costmap_2d cost_values.hpp from the C++ implementation,
defining the special cost values used throughout the costmap.
"""

from enum import IntEnum


class CombinationMethod(IntEnum):
    """Describe the method used to add data to the master costmap (default Max)."""

    # Every valid value from this layer is written into the master grid
    # (does not copy NO_INFORMATION).
    Overwrite = 0
    # Set the new value to the maximum of the master grid's value and this
    # layer's value. If the master value is NO_INFORMATION it is overwritten;
    # if the layer's value is NO_INFORMATION the master value does not change.
    Max = 1
    # Like Max but if the master value is NO_INFORMATION it is NOT overwritten.
    MaxWithoutUnknownOverwrite = 2


# No information / unknown cell
NO_INFORMATION: int = 255

# Lethal obstacle (definite collision)
LETHAL_OBSTACLE: int = 254

# Inscribed inflated obstacle (robot center at this cell → collision)
INSCRIBED_INFLATED_OBSTACLE: int = 253

# Maximum non-obstacle cost
MAX_NON_OBSTACLE: int = 252

# Free space
FREE_SPACE: int = 0
