#include "state_interpreter.hpp"

namespace state_interpreter
{

std::string StateInterpreter::getPathByFind(const std::string input_str)
{
  if (input_str.find("$(find ") == std::string::npos)
  {
    return input_str;
  }
  else
  {
    std::string split_str = "$(find ";
    int offset_num = input_str.find(split_str);
    std::string split_b = input_str.substr(offset_num + split_str.size());

    int end_num = split_b.find(")");
    std::string package_name = split_b.substr(0, end_num);

    int split_num = input_str.find(split_str + package_name + ")");
    std::string file_pos = split_b.substr(end_num + 1);

    return ros::package::getPath(package_name) + file_pos;
  }
}

std::string StateInterpreter::getPathByEnv(const std::string input_str)
{
  if (input_str.find("$(env ") == std::string::npos)
  {
    return input_str;
  }
  else
  {
    std::string split_str = "$(env ";
    int offset_num = input_str.find(split_str);
    std::string split_b = input_str.substr(offset_num + split_str.size());

    int end_num = split_b.find(")");
    std::string env_name = split_b.substr(0, end_num);

    int split_num = input_str.find(split_str + env_name + ")");
    std::string file_pos = split_b.substr(end_num + 1);
    std::cout << "env: " << std::getenv(env_name.c_str()) + file_pos << '\n';

    return std::getenv(env_name.c_str()) + file_pos;
  }
}

void StateInterpreter::initStateMapFromYaml(const std::string file_path)
{
  const YAML::Node StateYaml = YAML::LoadFile(file_path);

  for (int i=0; i<StateYaml.size(); i++)
  {
    StateInfo info;


    if (!static_cast<bool>(StateYaml[i]["text"]))
    {
      info.text = "Unknown";
    }
    else
    {
      info.text = StateYaml[i]["text"].as<std::string>();
    }

    if (!static_cast<bool>(StateYaml[i]["img_path"]))
    {
      info.img_path = "unknown_path";
    }
    else
    {
      info.img_path = StateYaml[i]["img_path"].as<std::string>();
      if (info.img_path.find("$(find ") != std::string::npos)
        info.img_path = getPathByFind(info.img_path);
      else if (info.img_path.find("$(env ") != std::string::npos)
        info.img_path = getPathByEnv(info.img_path);
    }

    if (!static_cast<bool>(StateYaml[i]["sound_bgm"]))
    {
      info.sound_bgm.path = "";
      info.sound_bgm.loop = false;
    }
    else
    {
      info.sound_bgm.path = StateYaml[i]["sound_bgm"]["path"].as<std::string>();
      info.sound_bgm.loop = StateYaml[i]["sound_bgm"]["loop"].as<bool>();
      if (info.sound_bgm.path.find("$(find ") != std::string::npos)
        info.sound_bgm.path = getPathByFind(info.sound_bgm.path);
      else if (info.sound_bgm.path.find("$(env ") != std::string::npos)
        info.sound_bgm.path = getPathByEnv(info.sound_bgm.path);
    }

    if (!static_cast<bool>(StateYaml[i]["sound_voice"]))
    {
      info.sound_voice.path = "";
      info.sound_voice.loop = false;
    }
    else
    {
      info.sound_voice.path = StateYaml[i]["sound_voice"]["path"].as<std::string>();
      info.sound_voice.loop = StateYaml[i]["sound_voice"]["loop"].as<bool>();
      if (info.sound_voice.path.find("$(find ") != std::string::npos)
        info.sound_voice.path = getPathByFind(info.sound_voice.path);
      else if (info.sound_voice.path.find("$(env ") != std::string::npos)
        info.sound_voice.path = getPathByEnv(info.sound_voice.path);
    }

    state_map_[StateYaml[i]["state"].as<std::string>()] = info;
  }

  StateInfo null_info;
  state_map_["Null"] = null_info;
}

void StateInterpreter::updateStopFactor()
{
  stop_factor_ = NONE;

  if (obstacle_waypoint_ != -1 && stopline_waypoint_ != -1)
  {
    if (obstacle_waypoint_ >= stopline_waypoint_)
      stop_factor_ = OBSTACLE;
    else
      stop_factor_ = STOPLINE;
  }
  else if (obstacle_waypoint_ != -1)
  {
    stop_factor_ = OBSTACLE;
  }
  else if (stopline_waypoint_ != -1)
  {
    stop_factor_ = STOPLINE;
  }
}

void StateInterpreter::callbackFromDecisionMakerState(const std_msgs::String& msg)
{
  decision_maker_state_ = msg.data;
}

void StateInterpreter::callbackFromObstacleWaypoint(const std_msgs::Int32& msg)
{
  obstacle_waypoint_ = msg.data;
}

void StateInterpreter::callbackFromStoplineWaypoint(const std_msgs::Int32& msg)
{
  stopline_waypoint_ = msg.data;
}

void StateInterpreter::callbackFromCurrentVelocity(const geometry_msgs::TwistStamped& msg)
{
  current_velocity_ = msg.twist.linear.x;
}

void StateInterpreter::callbackFromDiagnostics(const diagnostic_msgs::DiagnosticArray& msg)
{
  for (auto& status : msg.status)
  {
    if (status.name == "sound_bgm/soundplay_node: Node State")
    {
      for (auto& values : status.values)
      {
        if (values.key == "Active sounds")
        {
          if (values.value == "0")
            sound_end_ = true;
          else
            sound_end_ = false;
          received_bgm_diag_ = true;
          break;
        }
      }
      break;
    }
    else if (status.name == "sound_voice/soundplay_node: Node State")
    {
      for (auto& values : status.values)
      {
        if (values.key == "Active sounds")
        {
          if (values.value == "0")
            voice_end_ = true;
          else
            voice_end_ = false;
          received_voice_diag_ = true;
          break;
        }
      }
      break;
    }
  }
}

void StateInterpreter::stateUpdate()
{
  updateStopFactor();

  static std::string prev_state = "Null";
  static bool is_engaging = false;
  static unsigned int engaging_count = 0;
  constexpr unsigned int max_engaging_count = 10;

  if (decision_maker_state_.find("RightTurn") != std::string::npos)
  {
    current_state_blinker_ = "RightTurn";
  }
  else if (decision_maker_state_.find("LeftTurn") != std::string::npos)
  {
    current_state_blinker_ = "LeftTurn";
  }
  else
  {
    current_state_blinker_ = "Straight";
  }

  if (decision_maker_state_.find("\nDriving\n") != std::string::npos)
  {
    current_state_second_ = "Driving";
    current_state_code_.data = 300;
    if (prev_state.find("\nDriveReady\n") != std::string::npos)
    {
      is_engaging = true;
    }
  }
  if (current_velocity_ < 0.1)
  {
    current_state_second_ = "Stopping";
    current_state_code_.data = 400;
  }
  if (decision_maker_state_ == "")
  {
    current_state_second_ = "Null";
  }

  if (is_engaging)
  {
    current_state_first_ = "Engaging";
    current_state_code_.data = 301;
    engaging_count++;
    if (engaging_count >= max_engaging_count)
    {
      is_engaging = false;
      engaging_count = 0;
    }
  }
  else if (decision_maker_state_.find("Init\n") != std::string::npos)
  {
    current_state_first_ = "Init";
    current_state_code_.data = 101;
  }
  else if (decision_maker_state_.find("Charging\n") != std::string::npos)
  {
    current_state_first_ = "Charging";
    current_state_code_.data = 201;
  }
  else if (decision_maker_state_.find("Emergency\n") != std::string::npos)
  {
    current_state_first_ = "Emergency";
    current_state_code_.data = 601;
  }
  else if (decision_maker_state_.find("WaitOrder\n") != std::string::npos
        || decision_maker_state_.find("MissionCheck\n") != std::string::npos)
        {
    current_state_first_ = "RouteReceving";
    current_state_code_.data = 202;
  }
  else if (decision_maker_state_.find("\nDriveReady\n") != std::string::npos)
  {
    current_state_first_ = "WaitEngage";
    current_state_code_.data = 203;
  }
  else if (decision_maker_state_.find("MissionComplete\n") != std::string::npos)
  {
    current_state_first_ = "Arrived";
    current_state_code_.data = 501;
  }
  else if (decision_maker_state_.find("StopLine\n") != std::string::npos && stop_factor_ == STOPLINE)
  {
    current_state_first_ = "Stopline";
    current_state_code_.data += 2;
  }
  else if (decision_maker_state_.find("Go\n") != std::string::npos
        && stop_factor_ == OBSTACLE)
        {
    current_state_first_ = "ObstacleDetecting";
    current_state_code_.data += 3;
  }
  else if(decision_maker_state_.find("\nDriving\n") != std::string::npos && current_velocity_ != 0.0)
  {
    current_state_first_ = "Null";
  }

  if (current_state_first_ == "WaitEngage"
        && stop_factor_ == OBSTACLE)
  {
    current_state_first_ = "ObstacleDetecting";
  }


  if (state_map_.count(current_state_first_) == 0)
  {
    ROS_ERROR_STREAM(current_state_first_ << " state does not exist.");
    current_state_first_ = "Null";
  }
  else
  {
    upper_state_img_path_.data = state_map_.at(current_state_first_).img_path;
  }

  if (state_map_.count(current_state_second_) == 0)
  {
    ROS_ERROR_STREAM(current_state_second_ << " state does not exist.");
    current_state_second_ = "Null";
  }
  else
  {
    lower_state_img_path_.data = state_map_.at(current_state_second_).img_path;
  }

  state_text_.data = state_map_.at(current_state_first_).text + "\n" + state_map_.at(current_state_second_).text;

  current_state_code_.header.stamp = ros::Time::now();
  prev_state = decision_maker_state_;
}

void StateInterpreter::soundBgmUpdate()
{
  if (state_map_.count(current_state_second_) == 0)
  {
    ROS_ERROR_STREAM(current_state_second_ << " state does not exist.");
    return;
  }

  static bool sound_playing = false;
  const double loop_timeout = 3.0;
  static ros::Time loop_time = ros::Time::now();
  const double duration = (ros::Time::now() - loop_time).toSec();

  std::string bgm_file_path = state_map_.at(current_state_second_).sound_bgm.path;

  if (sound_playing && sound_end_ && received_bgm_diag_ && duration > loop_timeout)
  {
    sound_playing = false;
    loop_time = ros::Time::now();
  }

  if (bgm_file_path == "")
  {
    if (sound_playing)
    {
      bgm_client_->stopAll();
      sound_playing = false;
    }
  }
  else if (!sound_playing && received_bgm_diag_)
  {
    bgm_client_->playWave(bgm_file_path, 1.0f);
    sound_playing = true;
    loop_time = ros::Time::now();
    received_bgm_diag_ = false;
  }
}

void StateInterpreter::soundVoiceUpdate()
{
  if (state_map_.count(current_state_first_) == 0)
  {
    ROS_ERROR_STREAM(current_state_first_ << " state does not exist.");
    return;
  }
  if (state_map_.count(current_state_blinker_) == 0)
  {
    ROS_ERROR_STREAM(current_state_blinker_ << " state does not exist.");
    return;
  }

  static bool voice_playing = false;
  static bool blinker_playing = false;
  static std::string prev_state = "";
  static std::string prev_blinker_state = "";
  std::string voice_file_path = state_map_.at(current_state_first_).sound_voice.path;
  std::string blinker_file_path = state_map_.at(current_state_blinker_).sound_voice.path;

  // reset playing flag
  if (current_state_first_ != prev_state)
  {
    voice_playing = false;
  }
  if (prev_blinker_state != current_state_blinker_)
  {
    blinker_playing = false;
  }

  // blinker voice
  if (blinker_file_path != "")
  {
    if (!blinker_playing || (state_map_.at(current_state_blinker_).sound_voice.loop && voice_end_ && received_voice_diag_))
    {
      voice_client_->playWave(blinker_file_path, 1.0f);
      blinker_playing = true;
      received_voice_diag_ = false;
    }
  }

  // other voice
  if (voice_file_path != "" && !blinker_playing)
  {
    if (!voice_playing || state_map_.at(current_state_first_).sound_voice.loop && voice_end_ && received_voice_diag_)
    {
      voice_client_->playWave(voice_file_path, 1.0f);
      voice_playing = true;
      received_voice_diag_ = false;
    }
  }

  prev_state = current_state_first_;
  prev_blinker_state = current_state_blinker_;
}

void StateInterpreter::run() {
  ros::Rate rate(10);
  while (ros::ok()) {

    stateUpdate();
    soundBgmUpdate();
    soundVoiceUpdate();

    text_pub.publish(state_text_);
    upper_img_path_pub.publish(upper_state_img_path_);
    lower_img_path_pub.publish(lower_state_img_path_);
    state_code_pub.publish(current_state_code_);

    ros::spinOnce();
    rate.sleep();
  }
}

} // namespace state_interpreter
