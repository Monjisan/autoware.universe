// Copyright 2022 Tier IV, Inc. All rights reserved.
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

#ifndef CIRCULAR_GRAPH_HPP_
#define CIRCULAR_GRAPH_HPP_

#include <boost/optional.hpp>

#include <cstddef>
#include <functional>
#include <limits>
#include <stack>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace auto_parking_planner
{

template <typename T>
using VecVec = std::vector<std::vector<T>>;

template <typename ElementT>
class CircularGraphBase
{
public:
  CircularGraphBase() : CircularGraphBase([](const std::vector<ElementT> &) { return true; }) {}
  explicit CircularGraphBase(
    const std::function<bool(const std::vector<ElementT> &)> & f_is_stoppable)
  : f_is_stoppable_trajectory_(f_is_stoppable)
  {
  }
  bool hasLoop(const ElementT & element) const;
  VecVec<ElementT> planCircularPathSequence(const ElementT & element) const;

  virtual ~CircularGraphBase() = default;

protected:
  std::function<bool(const std::vector<ElementT> &)> f_is_stoppable_trajectory_;

private:
  std::vector<ElementT> computeEntireCircularPathWithoutLoop(const ElementT & element) const;
  std::vector<ElementT> computeEntireCircularPathWithLoop(const ElementT & element) const;
  VecVec<ElementT> splitPathContainingLoop(const std::vector<ElementT> & elem_path) const;
  virtual std::vector<ElementT> getFollowings(const ElementT & element) const = 0;
  virtual std::vector<ElementT> getReachables(const ElementT & element) const = 0;
  virtual size_t getID(const ElementT & element) const = 0;
  virtual size_t getElementNum() const = 0;
};

template <typename ElementT>
bool CircularGraphBase<ElementT>::hasLoop(const ElementT & element) const
{
  std::unordered_set<size_t> visit_set;
  std::stack<ElementT> s;
  s.push(element);
  while (!s.empty()) {
    const auto elem_here = s.top();
    s.pop();
    const bool is_visisted = visit_set.find(getID(elem_here)) != visit_set.end();
    if (is_visisted) return true;

    visit_set.insert(getID(elem_here));
    const auto elmes_following = getFollowings(elem_here);
    for (const auto elem : elmes_following) {
      s.push(elem);
    }
  }
  return false;
}

template <typename ElementT>
VecVec<ElementT> CircularGraphBase<ElementT>::planCircularPathSequence(
  const ElementT & element) const
{
  if (hasLoop(element)) {
    const auto entire_path_seq = computeEntireCircularPathWithLoop(element);
    return splitPathContainingLoop(entire_path_seq);
  } else {
    const auto no_loop_path = computeEntireCircularPathWithoutLoop(element);
    return VecVec<ElementT>{no_loop_path};
  }

  // No loop case
}

template <typename ElementT>
std::vector<ElementT> CircularGraphBase<ElementT>::computeEntireCircularPathWithoutLoop(
  const ElementT & element) const
{
  ElementT elem_next = element;
  std::vector<ElementT> no_loop_path{elem_next};
  while (true) {
    const auto elems_cand = getFollowings(elem_next);
    if (elems_cand.empty()) {
      break;
    }
    elem_next = elems_cand.front();  // TODO: case there are more than 1 deadends
    no_loop_path.push_back(elem_next);
  }

  // rewind if terminal of the partial path is not stoppable
  while (true) {
    if (f_is_stoppable_trajectory_(no_loop_path)) break;
    no_loop_path.pop_back();
  }
  return no_loop_path;
}

template <typename ElementT>
std::vector<ElementT> CircularGraphBase<ElementT>::computeEntireCircularPathWithLoop(
  const ElementT & element) const
{
  std::unordered_set<size_t> outside_of_loop_set;
  std::unordered_map<size_t, size_t> visit_counts;
  std::vector<ElementT> circular_path;
  const size_t n_reachable = getReachables(element).size();

  // lambda
  const auto isOutOfLoop = [&](const ElementT & elem) -> bool {
    if (outside_of_loop_set.find(getID(elem)) != outside_of_loop_set.end()) return true;
    if (hasLoop(elem)) return false;

    for (const auto & elem_descendants : getReachables(elem)) {
      outside_of_loop_set.insert(getID(elem_descendants));
    }
    return true;
  };

  // lambda
  const auto getVisitCount = [&](const ElementT & elem) -> size_t {
    const auto id = getID(elem);
    const bool never_visit = visit_counts.find(id) == visit_counts.end();
    if (never_visit) visit_counts[id] = 0;
    return visit_counts[id];
  };

  // lambda
  const auto incrementVisitCount = [&](const ElementT & elem) -> void {
    const auto id = getID(elem);
    const bool never_visit = visit_counts.find(id) == visit_counts.end();
    if (never_visit) visit_counts[id] = 0;
    visit_counts[id] += 1;
  };

  // lambda
  const auto markedAllNode = [&]() -> bool {
    size_t c = 0;
    for (const auto p : visit_counts) {
      if (p.second > 0) c++;
    }
    const auto n_total_mark = c + outside_of_loop_set.size();
    if (n_total_mark > n_reachable) throw std::logic_error("strange");
    return (n_total_mark == n_reachable);
  };

  // main
  ElementT element_here = element;
  while (!markedAllNode()) {
    incrementVisitCount(element_here);
    circular_path.push_back(element_here);

    const auto elems_child = getFollowings(element_here);

    ElementT element_next;
    const bool is_forking = elems_child.size() > 1;
    if (!is_forking) {
      const auto elem_next_cand = elems_child.front();
      if (isOutOfLoop(elem_next_cand)) throw std::logic_error("strange");
      element_next = elem_next_cand;
    } else {
      boost::optional<size_t> best_idx = boost::none;
      size_t min_visit_count = std::numeric_limits<size_t>::max();
      for (size_t idx = 0; idx < elems_child.size(); ++idx) {
        const auto & elem_next = elems_child.at(idx);

        if (isOutOfLoop(elem_next)) continue;

        const auto visit_count = getVisitCount(elem_next);
        if (visit_count < min_visit_count) {
          best_idx = idx;
          min_visit_count = visit_count;
        }
      }
      element_next = elems_child[best_idx.get()];
    }
    element_here = element_next;
  }
  return circular_path;
}

template <typename ElementT>
VecVec<ElementT> CircularGraphBase<ElementT>::splitPathContainingLoop(
  const std::vector<ElementT> & elem_path) const
{
  std::vector<std::vector<ElementT>> partial_path_seq;

  std::vector<ElementT> partial_path;

  const auto will_form_loop = [this, &partial_path](const ElementT & elem_here) {
    const size_t id_here = getID(elem_here);
    for (const auto & elem : partial_path) {
      if (getID(elem) == id_here) {
        return true;
      }
    }
    return false;
  };

  for (const auto & elem : elem_path) {
    if (will_form_loop(elem)) {  // split the loop!

      // Initialize partial_path
      auto partial_path_new = std::vector<ElementT>{partial_path.back()};  // Later will be reversed

      // rewind if terminal of the partial path is not stoppable
      while (true) {
        if (f_is_stoppable_trajectory_(partial_path)) break;
        partial_path.pop_back();
        partial_path_new.push_back(partial_path.back());
      }
      partial_path_seq.push_back(partial_path);
      std::reverse(partial_path_new.begin(), partial_path_new.end());
      partial_path = partial_path_new;
    }
    partial_path.push_back(elem);
  }

  while (true) {
    // rewind if terminal of the partial path is not stoppable
    if (f_is_stoppable_trajectory_(partial_path)) break;
    partial_path.pop_back();
  }
  partial_path_seq.push_back(partial_path);
  return partial_path_seq;
}

}  // namespace auto_parking_planner

#endif  // CIRCULAR_GRAPH_HPP_