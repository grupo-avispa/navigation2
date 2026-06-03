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

# No information / unknown cell
NO_INFORMATION: int = 255

# Lethal obstacle (definite collision)
LETHAL_OBSTACLE: int = 254

# Inscribed inflated obstacle (robot center at this cell → collision)
INSCRIBED_INFLATED_OBSTACLE: int = 253

# Free space
FREE_SPACE: int = 0

# Maximum non-lethal cost
MAX_NON_LETHAL: int = 252
