/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file behavior_planner_FSM.cpp
 **/

#include "behavior_planner_FSM.h"

State BehaviorPlannerFSM::get_closest_waypoint_goal(
    const State& ego_state, const SharedPtr<cc::Map>& map,
    const float& lookahead_distance, bool& is_goal_junction) {
  // Nearest waypoint on the center of a Driving Lane.
  auto waypoint_0 = map->GetWaypoint(ego_state.location);

  if (_active_maneuver == DECEL_TO_STOP || _active_maneuver == STOPPED) {
    State waypoint;
    auto wp_transform = waypoint_0->GetTransform();
    waypoint.location = wp_transform.location;
    waypoint.rotation.yaw = utils::deg2rad(wp_transform.rotation.yaw);
    waypoint.rotation.pitch = utils::deg2rad(wp_transform.rotation.pitch);
    waypoint.rotation.roll = utils::deg2rad(wp_transform.rotation.roll);
    return waypoint;
  }

  // Waypoints at a lookahead distance
  // NOTE: "GetNext(d)" creates a list of waypoints at an approximate distance
  // "d" in the direction of the lane. The list contains one waypoint for each
  // deviation possible.

  // NOTE 2: GetNextUntilLaneEnd(d) returns a list of waypoints a distance "d"
  // apart. The list goes from the current waypoint to the end of its
  // lane.

  auto lookahead_waypoints = waypoint_0->GetNext(lookahead_distance);
  auto n_wp = lookahead_waypoints.size();
  if (n_wp == 0) {
    // LOG(INFO) << "Goal wp is a nullptr";
    State waypoint;
    return waypoint;
  }
  // LOG(INFO) << "BP - Num of Lookahead waypoints: " << n_wp;

  waypoint_0 = lookahead_waypoints[lookahead_waypoints.size() - 1];

  is_goal_junction = waypoint_0->IsJunction();
  // LOG(INFO) << "BP - Is Last wp in junction? (0/1): " << is_goal_junction;
  auto cur_junction_id = waypoint_0->GetJunctionId();
  if (is_goal_junction) {
    if (cur_junction_id == _prev_junction_id) {
      // LOG(INFO) << "BP - Last wp is in same junction as ego. Junction ID: "
      //          << _prev_junction_id;
      is_goal_junction = false;
    } else {
      // LOG(INFO) << "BP - Last wp is in different junction than ego. Junction
      // ID: "
      //          << cur_junction_id;
      _prev_junction_id = cur_junction_id;
    }
  }
  State waypoint;
  auto wp_transform = waypoint_0->GetTransform();
  waypoint.location = wp_transform.location;
  waypoint.rotation.yaw = utils::deg2rad(wp_transform.rotation.yaw);
  waypoint.rotation.pitch = utils::deg2rad(wp_transform.rotation.pitch);
  waypoint.rotation.roll = utils::deg2rad(wp_transform.rotation.roll);
  return waypoint;
}

double BehaviorPlannerFSM::get_look_ahead_distance(const State& ego_state) {
  auto velocity_mag = utils::magnitude(ego_state.velocity);
  auto accel_mag = utils::magnitude(ego_state.acceleration);

  // TODO-Lookahead: One way to find a reasonable lookahead distance is to find
  // the distance you will need to come to a stop while traveling at speed V and
  // using a comfortable deceleration.
  // #001 TODO 妥当な先読み距離の計算
  // 妥当な先読み距離を見つける1つの方法は、速度Vで走行し、
  // 快適な減速を使用しているときに停止する必要がある距離を見つけることです。

  /*  単位時間の変化と仮定： dd = a*(1^2) + v*1  */
  auto look_ahead_distance = velocity_mag + accel_mag;  // <- Fix This	#001

  // LOG(INFO) << "Calculated look_ahead_distance: " << look_ahead_distance;

  look_ahead_distance =
      std::min(std::max(look_ahead_distance, _lookahead_distance_min),
               _lookahead_distance_max);

  // LOG(INFO) << "Final look_ahead_distance: " << look_ahead_distance;

  return look_ahead_distance;
}

State BehaviorPlannerFSM::get_goal(const State& ego_state,
                                   SharedPtr<cc::Map> map) {
  // Get look-ahead distance based on Ego speed

  auto look_ahead_distance = get_look_ahead_distance(ego_state);

  // Nearest waypoint on the center of a Driving Lane.
  bool is_goal_in_junction{false};
  auto goal_wp = get_closest_waypoint_goal(ego_state, map, look_ahead_distance,
                                           is_goal_in_junction);

  // LOG(INFO) << "Is the FINAL goal on a junction: " << is_goal_in_junction;
  string tl_state = "none";
  State goal =
      state_transition(ego_state, goal_wp, is_goal_in_junction, tl_state);

  return goal;
}

State BehaviorPlannerFSM::state_transition(const State& ego_state, State goal,
                                           bool& is_goal_in_junction,
                                           string tl_state) {
  // Check with the Behavior Planner to see what we are going to do and
  // where our next goal is
  //

  goal.acceleration.x = 0;
  goal.acceleration.y = 0;
  goal.acceleration.z = 0;

  /*  動作：レーン維持  */
  if (_active_maneuver == FOLLOW_LANE) {
    // LOG(INFO) << "BP- IN FOLLOW_LANE STATE";
    /*  交差点のゴール内にいる  */
    if (is_goal_in_junction) {
      // LOG(INFO) << "BP - goal in junction";

      _active_maneuver = DECEL_TO_STOP;
      // LOG(INFO) << "BP - changing to DECEL_TO_STOP";

      // Let's backup a "buffer" distance behind the "STOP" point
      // LOG(INFO) << "BP- original STOP goal at: " << goal.location.x << ", "
      //          << goal.location.y;

      // TODO-goal behind the stopping point: put the goal behind the stopping
      // point (i.e the actual goal location) by "_stop_line_buffer". HINTS:
      // remember that we need to go back in the opposite direction of the
      // goal/road, i.e you should use: ang = goal.rotation.yaw + M_PI and then
      // use cosine and sine to get x and y
      //
      //// #002 TODO 停止点の後ろの目標：
      //「_ stop_line_buffer」によって停止点の後ろ（つまり、実際の目標位置）に目標を置きます。
      // ヒント：ゴール/道路の反対方向に戻る必要があることを忘れないでください。
      // つまり、ang = goal.rotation.yaw + M_PIを使用してから、
      // コサインとサインを使用してxとyを取得する必要があります。
      /*  目標位置 = 停止線バッファ * yaw角の影響  */
      auto ang = goal.rotation.yaw + M_PI;
      goal.location.x += _stop_line_buffer * std::cos(ang);  // <- Fix This #002
      goal.location.y += _stop_line_buffer * std::sin(ang);  // <- Fix This #002

      // LOG(INFO) << "BP- new STOP goal at: " << goal.location.x << ", "
      //          << goal.location.y;

      // TODO-goal speed at stopping point: What should be the goal speed??
      // #003 TODO-停止点での目標速度：目標速度はどうあるべきですか？
      goal.velocity.x = 0.0;  // <- Fix This #003
      goal.velocity.y = 0.0;  // <- Fix This #003
      goal.velocity.z = 0.0;  // <- Fix This #003

    } else {
      // TODO-goal speed in nominal state: What should be the goal speed now
      // that we know we are in nominal state and we can continue freely?
      // Remember that the speed is a vector
      // HINT: _speed_limit * std::sin/cos (goal.rotation.yaw);
      // #004 TODO-公称状態での目標速度：公称状態にあり、自由に続行できることがわかったので、
      // 目標速度はどうなりますか？
      // 速度はベクトルであることを忘れないでください。
      // ヒント：_speed_limit * std :: sin / cos（goal.rotation.yaw）;
      goal.velocity.x = _speed_limit * std::cos(goal.rotation.yaw);  // <- Fix This #004
      goal.velocity.y = _speed_limit * std::sin(goal.rotation.yaw);  // <- Fix This #004
      goal.velocity.z = 0;
    }
  /*  動作：ストップ  */
  } else if (_active_maneuver == DECEL_TO_STOP) {
    // LOG(INFO) << "BP- IN DECEL_TO_STOP STATE";
    // TODO-maintain the same goal when in DECEL_TO_STOP state: Make sure the
    // new goal is the same as the previous goal (_goal). That way we
    // keep/maintain the goal at the stop line.
    // #005 TODO-DECEL_TO_STOP状態のときに同じ目標を維持します：
    // 新しい目標が前の目標（_goal）と同じであることを確認します。
    // そうすれば、ストップラインでゴールを維持/維持できます。

    /*  前回目標値を取得。この関数の末尾でも実施しているため不要？？  */
      goal = _goal;  // <- Fix This #005

    // TODO: It turns out that when we teleport, the car is always at speed
    // zero. In this the case, as soon as we enter the DECEL_TO_STOP state,
    // the condition that we are <= _stop_threshold_speed is ALWAYS true and we
    // move straight to "STOPPED" state. To solve this issue (since we don't
    // have a motion controller yet), you should use "distance" instead of
    // speed. Make sure the distance to the stopping point is <=
    // P_STOP_THRESHOLD_DISTANCE. Uncomment the line used to calculate the
    // distance
    // #006 TODO：テレポートすると、車の速度は常にゼロになります。
    // この場合、DECEL_TO_STOP状態に入るとすぐに、<= _stop_threshold_speedであるという条件は常に真であり、
    // 「STOPPED」状態に直接移行します。
    // この問題を解決するには（モーションコントローラがまだないため）、
    // 速度ではなく「距離」を使用する必要があります。
    // 停止点までの距離が<= P_STOP_THRESHOLD_DISTANCEであることを確認してください。
    // 距離の計算に使用した線のコメントを外します
    auto distance_to_stop_sign =
        utils::magnitude(goal.location - ego_state.location);
    // LOG(INFO) << "Ego distance to stop line: " << distance_to_stop_sign;

    // TODO-use distance rather than speed: Use distance rather than speed...
    // #006-1 TODO-速度ではなく距離を使用する：速度ではなく距離を使用します...
//    if (utils::magnitude(ego_state.velocity) <=
//        _stop_threshold_speed) {  // -> Fix this
      if (distance_to_stop_sign <= P_STOP_THRESHOLD_DISTANCE) {     /*  #006-1  */
      // TODO-move to STOPPED state: Now that we know we are close or at the
      // stopping point we should change state to "STOPPED"
      // #006-2 TODO-STOPPED状態に移行する：
      // 接近している、または停止点にあることがわかったので、状態を「STOPPED」に変更する必要があります
      _active_maneuver = STOPPED;  // <- Fix This #006-2
      _start_stop_time = std::chrono::high_resolution_clock::now();
      // LOG(INFO) << "BP - changing to STOPPED";
    }
  } else if (_active_maneuver == STOPPED) {
    // LOG(INFO) << "BP- IN STOPPED STATE";
    // TODO-maintain the same goal when in STOPPED state: Make sure the new goal
    // is the same as the previous goal. That way we keep/maintain the goal at
    // the stop line. goal = ...;
    // #007 TODO-STOPPED状態のときに同じ目標を維持する：
    // 新しい目標が前の目標と同じであることを確認します。
    // そうすれば、ストップラインでゴールを維持/維持できます。目標= ...;
      goal = _goal;  // Keep previous goal. Stay where you are. // <- Fix This #007

    long long stopped_secs =
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::high_resolution_clock::now() - _start_stop_time)
            .count();
    // LOG(INFO) << "BP- Stopped for " << stopped_secs << " secs";

    if (stopped_secs >= _req_stop_time && tl_state.compare("Red") != 0) {
      // TODO-move to FOLLOW_LANE state: What state do we want to move to, when
      // we are "done" at the STOPPED state?
      // #008 TODO-FOLLOW_LANE状態に移行：STOPPED状態で「完了」したときに、どの状態に移行しますか？
      _active_maneuver = FOLLOW_LANE;  // <- Fix This #008
      // LOG(INFO) << "BP - changing to FOLLOW_LANE";
    }
  }
  _goal = goal;
  return goal;
}
