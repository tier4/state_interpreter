#include "state_interpreter.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "state_interpreter");
  state_interpreter::StateInterpreter state_interpreter_node;
  state_interpreter_node.run();
  // ros::spin();
  return 0;
}
