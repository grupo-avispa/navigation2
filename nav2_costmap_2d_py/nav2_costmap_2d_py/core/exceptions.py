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
Exceptions for nav2_costmap_2d_py.

It mirrors the nav2_costmap_2d exceptions.hpp from the C++ implementation.
"""


class CollisionCheckerException(RuntimeError):
    """Exception thrown if the collision checker determines a pose is in collision."""

    def __init__(self, description: str) -> None:
        """
        Construct the exception.

        Parameters
        ----------
        description : str
            Human-readable description of the collision error.

        """
        super().__init__(description)


class IllegalPoseException(CollisionCheckerException):
    """Exception thrown when the collision checker encounters a fatal error."""

    def __init__(self, name: str, description: str) -> None:
        """
        Construct the exception.

        Parameters
        ----------
        name : str
            Name of the critic/component that raised the exception.
        description : str
            Human-readable description of the error.

        """
        super().__init__(description)
        self._name = name

    def get_critic_name(self) -> str:
        """
        Return the name of the critic/component that raised the exception.

        Returns
        -------
        str
            The critic name.

        """
        return self._name
