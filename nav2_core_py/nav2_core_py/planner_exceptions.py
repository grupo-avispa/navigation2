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
Exceptions for Nav2 planners.

All exceptions accept a description message to provide context about
what went wrong during planning.

It mirrors the nav2_core::PlannerException from the C++ implementation.
"""


class PlannerException(RuntimeError):
    """Base class for all Nav2 planner exceptions."""

    def __init__(self, description: str):
        """
        Initialize the planner exception.

        Parameters
        ----------
        description : str
            A description of what went wrong.
        """
        super().__init__(description)
        self.description = description


class InvalidPlanner(PlannerException):
    """Raised when the requested planner plugin ID does not exist."""


class StartOccupied(PlannerException):
    """Raised when the start pose is inside an obstacle."""


class GoalOccupied(PlannerException):
    """Raised when the goal pose is inside an obstacle."""


class NoValidPathCouldBeFound(PlannerException):
    """Raised when the planner cannot find any valid path."""


class PlannerTimedOut(PlannerException):
    """Raised when the planner exceeds its allocated time budget."""


class StartOutsideMapBounds(PlannerException):
    """Raised when the start pose is outside the costmap bounds."""


class GoalOutsideMapBounds(PlannerException):
    """Raised when the goal pose is outside the costmap bounds."""


class TFError(PlannerException):
    """Raised on TF lookup failures during planning."""


class NoViableRoute(PlannerException):
    """Raised when no viable route exists between start and goal."""


class PlannerTFError(PlannerException):
    """Raised when a TF error occurs inside the planner plugin."""


class NoViapointsGiven(PlannerException):
    """Raised when no viapoints or goals are provided for planning."""


class PlannerCancelled(PlannerException):
    """Raised when the planning is cancelled by the user."""
