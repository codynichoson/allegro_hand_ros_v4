#include "nuallegro_node.h"
#include "bhand/BHand.h"

#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)

// The only topic specific to the 'grasp' controller is the envelop torque.
const std::string ENVELOP_TORQUE_TOPIC = "allegroHand/envelop_torque";

double home_pose[DOF_JOINTS] =
        {
                // Default (HOME) position (degrees), set at system start if
                // no 'initial_position.yaml' parameter is loaded.
                0.0, -10.0, 45.0, 45.0,  0.0, -10.0, 45.0, 45.0,
                5.0, -5.0, 50.0, 45.0, 60.0, 25.0, 15.0, 45.0
        };

NUAllegroNode::NUAllegroNode()
        : AllegroNode() {

  initController(whichHand);

  joint_cmd_sub = nh.subscribe(
          DESIRED_STATE_TOPIC, 3, &NUAllegroNode::setJointCallback, this);
  lib_cmd_sub = nh.subscribe(
          LIB_CMD_TOPIC, 1, &NUAllegroNode::libCmdCallback, this);

  envelop_torque_sub = nh.subscribe(
          ENVELOP_TORQUE_TOPIC, 1, &NUAllegroNode::envelopTorqueCallback,
          this);
}

NUAllegroNode::~NUAllegroNode() {
  delete pBHand;
}

void NUAllegroNode::computeDesiredTorque() {
  // compute control torque using Bhand library
  pBHand->SetJointPosition(current_position_filtered);

  // BHand lib control updated with time stamp
  pBHand->UpdateControl((double) frame * ALLEGRO_CONTROL_TIME_INTERVAL);

  // Necessary torque obtained from Bhand lib
  pBHand->GetJointTorque(desired_torque);

  //ROS_INFO("desired torque = %.3f %.3f %.3f %.3f", desired_torque[0], desired_torque[1], desired_torque[2], desired_torque[3]);
}

void NUAllegroNode::initController(const std::string &whichHand) {
  // Initialize BHand controller
  if (whichHand.compare("left") == 0) {
    pBHand = new BHand(eHandType_Left);
    ROS_WARN("CTRL: Left Allegro Hand controller initialized.");
  }
  else {
    pBHand = new BHand(eHandType_Right);
    ROS_WARN("CTRL: Right Allegro Hand controller initialized.");
  }
  pBHand->SetTimeInterval(ALLEGRO_CONTROL_TIME_INTERVAL);
  pBHand->SetMotionType(eMotionType_NONE);

  // sets initial desired pos at start pos for PD control
  for (int i = 0; i < DOF_JOINTS; i++)
    desired_position[i] = current_position[i];

  printf("*************************************\n");
  printf("          NU (BHand) Method          \n");
  printf("*************************************\n");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "allegro_hand_core_grasp");
  NUAllegroNode grasping;

  bool polling = false;
  if (argv[1] == std::string("true")) {
    polling = true;
  }
  ROS_INFO("Start controller with polling = %d", polling);

  for (int i = 0; i < DOF_JOINTS; i++)
      desired_joint_state.position[i] = DEGREES_TO_RADIANS(home_pose[i]);

  grasping.doIt(polling);
}