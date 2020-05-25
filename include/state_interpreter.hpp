/*
 * Copyright 2019-2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __STATE_INTERPRETER_HPP__
#define __STATE_INTERPRETER_HPP__

#include <ros/ros.h>
#include <ros/package.h>
#include <sound_play/sound_play.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <map>
#include <sstream>

#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include "state_interpreter/Int32Stamped.h"

namespace state_interpreter
{

struct SoundInfo {
  std::string path;
  bool loop;

  SoundInfo(void) : path(""), loop(false)
  {
  }
};

struct StateInfo {
  std::string text;
  std::string img_path;
  SoundInfo sound_bgm;
  SoundInfo sound_voice;

  StateInfo(void) : text(""), img_path("")
  {
  }
};

enum StopFactor {
  NONE,
  OBSTACLE,
  STOPLINE,
  OTHER
};

class StateInterpreter {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher text_pub, upper_img_path_pub, lower_img_path_pub, state_code_pub;
  ros::Subscriber state_sub, obstacle_waypoint_sub_, stopline_waypoint_sub_, current_vel_sub, diagnostics_sub_;
  std::unique_ptr<sound_play::SoundClient> bgm_client_;
  std::unique_ptr<sound_play::SoundClient> voice_client_;

  // private param
  std::map<std::string, StateInfo> state_map_;
  std::string current_state_first_;
  std::string current_state_second_;
  std::string current_state_blinker_;
  state_interpreter::Int32Stamped current_state_code_;
  std_msgs::String state_text_;
  std_msgs::String upper_state_img_path_;
  std_msgs::String lower_state_img_path_;
  // for callbacks
  bool sound_end_;
  bool voice_end_;
  bool received_bgm_diag_;
  bool received_voice_diag_;
  StopFactor stop_factor_;
  int obstacle_waypoint_;
  int stopline_waypoint_;
  double current_velocity_;
  std::string decision_maker_state_;

  // ros param
  std::string text_publish_topic_;
  std::string upper_img_path_publish_topic_;
  std::string lower_img_path_publish_topic_;
  std::string state_code_publish_topic_;
  std::string state_img_files_;
  std::string sound_bgm_topic_;
  std::string sound_voice_topic_;

  std::string getPathByFind(const std::string input_str);
  std::string getPathByEnv(const std::string input_str);
  void initStateMapFromYaml(const std::string file_path);
  void updateStopFactor();

  void callbackFromDecisionMakerState(const std_msgs::String& msg);
  void callbackFromStateCommand(const std_msgs::String& msg);
  void callbackFromObstacleWaypoint(const std_msgs::Int32& msg);
  void callbackFromStoplineWaypoint(const std_msgs::Int32& msg);
  void callbackFromCurrentVelocity(const geometry_msgs::TwistStamped& msg);
  void callbackFromDiagnostics(const diagnostic_msgs::DiagnosticArray& msg);
  void stateUpdate();
  void soundBgmUpdate();
  void soundVoiceUpdate();


public:
  StateInterpreter()
    : private_nh_("~")
    , current_state_first_("Null")
    , current_state_second_("Null")
    , current_state_blinker_("Straight")
    , sound_end_(false)
    , voice_end_(false)
    , received_bgm_diag_(false)
    , received_voice_diag_(false)
    , stop_factor_(NONE)
    , obstacle_waypoint_(-1)
    , stopline_waypoint_(-1)
    , current_velocity_(0.0)
    , decision_maker_state_("")
    , text_publish_topic_("/state_interpreter/state_text")
    , upper_img_path_publish_topic_("/state_interpreter/upper_state_img_path")
    , lower_img_path_publish_topic_("/state_interpreter/lower_state_img_path")
    , state_code_publish_topic_("/state_interpreter/state_code")
    , state_img_files_("")
    , sound_bgm_topic_("")
    , sound_voice_topic_("")
  {
    // rosparam
    private_nh_.getParam("text_publish_topic", text_publish_topic_);
    private_nh_.getParam("upper_img_path_publish_topic", upper_img_path_publish_topic_);
    private_nh_.getParam("lower_img_path_publish_topic", lower_img_path_publish_topic_);
    private_nh_.getParam("state_code_publish_topic", state_code_publish_topic_);
    private_nh_.getParam("state_img_files", state_img_files_);
    private_nh_.getParam("sound_bgm_topic", sound_bgm_topic_);
    private_nh_.getParam("sound_voice_topic", sound_voice_topic_);

    // sound client
    bgm_client_ = std::unique_ptr<sound_play::SoundClient>(new sound_play::SoundClient(nh_, sound_bgm_topic_));
    voice_client_ = std::unique_ptr<sound_play::SoundClient>(new sound_play::SoundClient(nh_, sound_voice_topic_));

    // subscriber
    state_sub = nh_.subscribe("/decision_maker/state", 1, &StateInterpreter::callbackFromDecisionMakerState, this);
    obstacle_waypoint_sub_ = nh_.subscribe("/obstacle_waypoint", 1, &StateInterpreter::callbackFromObstacleWaypoint, this);
    stopline_waypoint_sub_ = nh_.subscribe("/stopline_waypoint", 1, &StateInterpreter::callbackFromStoplineWaypoint, this);
    current_vel_sub = nh_.subscribe("/current_velocity", 1, &StateInterpreter::callbackFromCurrentVelocity, this);
    diagnostics_sub_ = nh_.subscribe("/diagnostics", 5, &StateInterpreter::callbackFromDiagnostics, this);

    // publisher
    text_pub = nh_.advertise<std_msgs::String>(text_publish_topic_, 1);
    upper_img_path_pub = nh_.advertise<std_msgs::String>(upper_img_path_publish_topic_, 1);
    lower_img_path_pub = nh_.advertise<std_msgs::String>(lower_img_path_publish_topic_, 1);
    state_code_pub = nh_.advertise<state_interpreter::Int32Stamped>(state_code_publish_topic_, 1);

    initStateMapFromYaml(state_img_files_);

    state_text_.data = "";
    upper_state_img_path_.data = "";
    lower_state_img_path_.data = "";
    current_state_code_.data = 0;
  };

  // ~StateInterpreter(){};

  void run();
};

} // namespace state_interpreter
#endif
