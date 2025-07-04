// Copyright (c) 2021, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef NAV2_SMAC_PLANNER__ANALYTIC_EXPANSION_HPP_
#define NAV2_SMAC_PLANNER__ANALYTIC_EXPANSION_HPP_

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <functional>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "nav2_smac_planner/node_2d.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/node_lattice.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/constants.hpp"

namespace nav2_smac_planner
{

template<typename NodeT>
class AnalyticExpansion
{
public:
  typedef NodeT * NodePtr;
  typedef std::vector<NodePtr> NodeVector;
  typedef typename NodeT::Coordinates Coordinates;
  typedef std::function<bool (const uint64_t &, NodeT * &)> NodeGetter;
  typedef typename NodeT::CoordinateVector CoordinateVector;

  /**
   * @struct nav2_smac_planner::AnalyticExpansion::AnalyticExpansionNodes
   * @brief Analytic expansion nodes and associated metadata
   */
  struct AnalyticExpansionNode
  {
    AnalyticExpansionNode(
      NodePtr & node_in,
      Coordinates & initial_coords_in,
      Coordinates & proposed_coords_in)
    : node(node_in),
      initial_coords(initial_coords_in),
      proposed_coords(proposed_coords_in)
    {
    }

    NodePtr node;
    Coordinates initial_coords;
    Coordinates proposed_coords;
  };

  /**
   * @struct AnalyticExpansionNodes
   * @brief Analytic expansion nodes and associated metadata
   *
   * This structure holds a collection of analytic expansion nodes and the number of direction
   * changes encountered during the expansion.
   */
  struct AnalyticExpansionNodes
  {
    AnalyticExpansionNodes() = default;

    void add(
      NodePtr & node,
      Coordinates & initial_coords,
      Coordinates & proposed_coords)
    {
      nodes.emplace_back(node, initial_coords, proposed_coords);
    }

    void setDirectionChanges(int changes)
    {
      direction_changes = changes;
    }

    std::vector<AnalyticExpansionNode> nodes;
    int direction_changes{0};
  };

  /**
   * @brief Constructor for analytic expansion object
   */
  AnalyticExpansion(
    const MotionModel & motion_model,
    const SearchInfo & search_info,
    const bool & traverse_unknown,
    const unsigned int & dim_3_size);

  /**
   * @brief Sets the collision checker and costmap to use in expansion validation
   * @param collision_checker Collision checker to use
   */
  void setCollisionChecker(GridCollisionChecker * collision_checker);

  /**
   * @brief Attempt an analytic path completion
   * @param node The node to start the analytic path from
   * @param coarse_check_goals Coarse list of goals nodes to plan to
   * @param fine_check_goals Fine list of goals nodes to plan to
   * @param goals_coords vector of goal coordinates to plan to
   * @param getter Gets a node at a set of coordinates
   * @param iterations Iterations to run over
   * @param closest_distance Closest distance to goal
   * @return Node pointer reference to goal node with the best score out of the goals node if
   * successful, else return nullptr
   */
  NodePtr tryAnalyticExpansion(
    const NodePtr & current_node,
    const NodeVector & coarse_check_goals,
    const NodeVector & fine_check_goals,
    const CoordinateVector & goals_coords,
    const NodeGetter & getter, int & iterations,
    int & closest_distance);

  /**
   * @brief Perform an analytic path expansion to the goal
   * @param node The node to start the analytic path from
   * @param goal The goal node to plan to
   * @param getter The function object that gets valid nodes from the graph
   * @param state_space State space to use for computing analytic expansions
   * @return A set of analytically expanded nodes to the goal from current node, if possible
   */
  AnalyticExpansionNodes getAnalyticPath(
    const NodePtr & node, const NodePtr & goal,
    const NodeGetter & getter, const ompl::base::StateSpacePtr & state_space);

  /**
   * @brief Refined analytic path from the current node to the goal
   * @param node The node to start the analytic path from. Node head may
   * change as a result of refinement
   * @param goal_node The goal node to plan to
   * @param getter The function object that gets valid nodes from the graph
   * @param analytic_nodes The set of analytic nodes to refine
   * @return The score of the refined path
   */
  float refineAnalyticPath(
    NodePtr & node,
    const NodePtr & goal_node,
    const NodeGetter & getter,
    AnalyticExpansionNodes & analytic_nodes);

  /**
   * @brief Takes final analytic expansion and appends to current expanded node
   * @param node The node to start the analytic path from
   * @param goal The goal node to plan to
   * @param expanded_nodes Expanded nodes to append to end of current search path
   * @return Node pointer to goal node if successful, else return nullptr
   */
  NodePtr setAnalyticPath(
    const NodePtr & node, const NodePtr & goal,
    const AnalyticExpansionNodes & expanded_nodes);

  /**
    * @brief Counts the number of direction changes in a Reeds-Shepp path
    * @param path The Reeds-Shepp path to count direction changes in
    * @return The number of direction changes in the path
    */
  int countDirectionChanges(const ompl::base::ReedsSheppStateSpace::ReedsSheppPath & path);

  /**
   * @brief Takes an expanded nodes to clean up, if necessary, of any state
   * information that may be polluting it from a prior search iteration
   * @param expanded_nodes Expanded node to clean up from search
   */
  void cleanNode(const NodePtr & nodes);

protected:
  MotionModel _motion_model;
  SearchInfo _search_info;
  bool _traverse_unknown;
  unsigned int _dim_3_size;
  GridCollisionChecker * _collision_checker;
  std::list<std::unique_ptr<NodeT>> _detached_nodes;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__ANALYTIC_EXPANSION_HPP_
