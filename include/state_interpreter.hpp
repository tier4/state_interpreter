#ifndef __STATE_INTERPRETER_HPP__
#define __STATE_INTERPRETER_HPP__

#include <ros/ros.h>
#include <ros/package.h>
#include <sound_play/sound_play.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <map>
#include <sstream>

#include <autoware_msgs/StopWaypointStatus.h>

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

class StateInterpreter {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher text_pub, upper_img_path_pub, lower_img_path_pub, state_code_pub;
  ros::Subscriber state_sub, stop_waypoint_sub, current_vel_sub, diagnostics_sub_;
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
  int stop_waypoint_type_;
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

  void initStateMapFromYaml(const std::string file_path);

  void callbackFromDecisionMakerState(const std_msgs::String& msg);
  void callbackFromStateCommand(const std_msgs::String& msg);
  void callbackFromStopWaypoint(const autoware_msgs::StopWaypointStatus& msg);
  void callbackFromCurrentVelocity(const geometry_msgs::TwistStamped& msg);
  void callbackFromDiagnostics(const diagnostic_msgs::DiagnosticArray& msg);
  void stateUpdate();
  void soundBgmUpdate();
  void soundVoiceUpdate();

  std::string getPathByFind(const std::string input_str);
  std::string getPathByEnv(const std::string input_str);

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
    , stop_waypoint_type_(0)
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
    stop_waypoint_sub = nh_.subscribe("/stop_waypoint_status", 1, &StateInterpreter::callbackFromStopWaypoint, this);
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
