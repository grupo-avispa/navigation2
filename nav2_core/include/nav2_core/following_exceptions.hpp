// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_CORE__FOLLOWING_EXCEPTIONS_HPP_
#define NAV2_CORE__FOLLOWING_EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>

namespace nav2_core
{

/**
 * @class FollowingException
 * @brief Abstract following exception
 */
class FollowingException : public std::runtime_error
{
public:
  explicit FollowingException(const std::string & description)
  : std::runtime_error(description) {}
};

/**
 * @class ObjectNotValid
 * @brief Action was invalid
 */
class ObjectNotValid : public FollowingException
{
public:
  explicit ObjectNotValid(const std::string & description)
  : FollowingException(description) {}
};

/**
 * @class FailedToDetectObject
 * @brief Failed to detect the object
 */
class FailedToDetectObject : public FollowingException
{
public:
  explicit FailedToDetectObject(const std::string & description)
  : FollowingException(description) {}
};

/**
 * @class FailedToControl
 * @brief Failed to control to the object
 */
class FailedToControl : public FollowingException
{
public:
  explicit FailedToControl(const std::string & description)
  : FollowingException(description) {}
};

}  // namespace nav2_core

#endif  // NAV2_CORE__FOLLOWING_EXCEPTIONS_HPP_