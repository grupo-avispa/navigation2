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
Exceptions for Nav2 controllers.

All exceptions accept a description message to provide context about
what went wrong during control computation.

It mirrors the nav2_core/controller_exceptions.hpp from the C++ implementation.
"""


class ControllerException(RuntimeError):
    """Base class for all Nav2 controller exceptions."""

    def __init__(self, description: str):
        """
        Initialize the controller exception.

        Parameters
        ----------
        description : str
            A description of what went wrong.

        """
        super().__init__(description)
        self.description = description


class InvalidController(ControllerException):
    """Raised when the requested controller plugin name does not exist."""


class ControllerTFError(ControllerException):
    """Raised when a required TF transform fails."""


class FailedToMakeProgress(ControllerException):
    """Raised when the robot is not making progress toward the goal."""


class PatienceExceeded(ControllerException):
    """Raised when the controller has been producing zero/invalid commands too long."""


class InvalidPath(ControllerException):
    """Raised when the received path is empty or malformed."""


class NoValidControl(ControllerException):
    """Raised when the controller could not produce a valid velocity command."""


class ControllerTimedOut(ControllerException):
    """Raised when costmap or other resource times out."""
