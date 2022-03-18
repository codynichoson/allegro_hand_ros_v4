#include "allegro_node_grasp.h"
#include "bhand/BHand.h"
 
// ROCK-SCISSORS-PAPER(RIGHT HAND)
static double rock[] = {
	-0.060443433046814224, 1.718152628368968, 1.5183127728064667, 1.0768768082224522, 
  -0.05366902617688447, 1.624485735198956, 1.5646042111477811, 1.0817334603483135, 
  -0.02420332445510799, 1.8175675509342855, 1.4558846409841664, 1.0296932444804807, 
  0.479222489188789, 0.8578085409952569, 0.4449529169021429, 1.7261426080470588};
static double paper[] = {
	-0.021832792449389456, -0.03035824187862029, 0.18801533593150305, 0.01198879646160758, 
  0.04413180310439969, -0.13971303534473847, 0.19187593317000584, 0.15814178583742344, 
  0.15880001917120196, 0.1233967956196272, 0.05858191722195744, 0.09848556244442067, 
  0.3681650319683363, 0.0384674312796147, 1.1671239111491531, 0.2975463032650292};
static double scissors[] = {
	-0.1638169947071259, -0.15219636633764633, 0.03298601399056162, 0.010027416704802557, 
  0.18359103183975056, -0.2594197379672195, 0.16419808618296794, 0.09120731524949205, 
  -0.05646432005434336, 1.8352313802175861, 1.4926448269164898, 0.7581361153450831, 
  0.6715169431147034, 0.2857890270681494, 0.48537797499374175, 1.6826323468467537};
static double bird[] = {
	-0.04803505353141109, 1.717778320220271, 1.4791653725457228, 1.1986978365600023, 
  0.054442538715400994, -0.11902311172969271, 0.1483218726749198, 0.20433246149861314, 
  0.113925353077646, 1.8350827783707961, 1.4035073329265262, 1.1615799688214161, 
  0.6715169431147034, 0.2857890270681494, 0.48537797499374175, 1.6826323468467537};
static double aloha[] = {
  -0.090635183088512, 1.602812419506062, 1.1404868878532919, 1.465613587602256, 
  -0.01034162310983767, 1.3913555552476224, 1.414896306052235, 1.5697152925895854, 
  0.15800900040688276, 0.04883952785254209, 0.13395057841064856, 0.22445919743062656, 
  0.4461814983303345, 0.2242174499852232, 0.15037444306507625, 0.7353577083999168};
static double horns[] = {
  -0.05668095741475797, -0.03455158204437531, 0.1421611862631179, 0.25145779253290335, 
  0.08665823077141369, 1.6244378469510439, 1.6415448565301425, 0.8124785992395168, 
  0.16865874457221036, 0.06851290024796633, 0.21068226587918185, -0.15489305101663145, 
  0.6715169431147034, 0.2857890270681494, 0.48537797499374175, 1.6826323468467537};
static double one[] = {
  -0.04530539772757582, -0.07754498652948032, 0.0847162418441131, 0.0567966266025635, 
  -0.0738810068316576, 1.623622577127107, 1.5763273358323262, 1.068693593294893, 
  -0.02283322550958373, 1.832048708689823, 1.4444579545758822, 0.991447506560065, 
  0.4773402978554376, 0.8189642488047152, 0.5055987638409126, 1.6655491913873082};
static double two[] = {
  0.03627480976682901, -0.11670200677142138, 0.08207319336335159, 0.05624047388269968, 
  -0.04029808539942264, -0.22051502864918018, 0.16448489070883873, 0.08451183308706563, 
  -0.023088440418527617, 1.8321956147009635, 1.4443688105214116, 0.9915045635930508, 
  0.477263820461654, 0.8189222494397977, 0.5056189762757072, 1.6657124748157996};
static double three[] = {
  0.03602605632514494, -0.11693968615039352, 0.08199298974337887, 0.056372223012127076, 
  -0.039941066932278134, -0.2212158101350734, 0.164398819111431, 0.08443808944742874, 
  0.0050197770809003546, 0.019177554183457332, 0.1272522055704284, -0.21176454685530827, 
  0.47751865437409846, 0.8188189627363381, 0.5055942701950735, 1.6656458630887414};
static double start[] = {
  -0.12479963620550248, 0.8610256881538054, 1.4924400634233945, 1.0186801734470297, 
  -0.025177060124453615, 0.7792930539390712, 1.5361886414553247, 1.1092395969692457, 
  0.08611417199542569, 0.9619508027474939, 1.4316393255357867, 1.0283443833084536, 
  0.4446351179435012, 0.8993441218990603, 0.38847355909899156, 1.7693947475908474};

// The only topic specific to the 'grasp' controller is the envelop torque.
const std::string ENVELOP_TORQUE_TOPIC = "allegroHand/envelop_torque";

// Define a map from string (received message) to eMotionType (Bhand controller grasp).
std::map<std::string, eMotionType> bhand_grasps = {
        {"home",     eMotionType_HOME},
        {"ready",    eMotionType_READY},  // ready position
        {"grasp_3",  eMotionType_GRASP_3},  // grasp with 3 fingers
        {"grasp_4",  eMotionType_GRASP_4},  // grasp with 4 fingers
        {"pinch_it", eMotionType_PINCH_IT},  // pinch, index & thumb
        {"pinch_mt", eMotionType_PINCH_MT},  // pinch, middle & thumb
        {"envelop",  eMotionType_ENVELOP},  // envelop grasp (power-y)
        {"off",      eMotionType_NONE},  // turn joints off
        {"gravcomp", eMotionType_GRAVITY_COMP},  // gravity compensation
        // These ones do not appear to do anything useful (or anything at all):
        // {"pregrasp", eMotionType_PRE_SHAPE},  // not sure what this is supposed to do.
        // {"move_object", eMotionType_OBJECT_MOVING},
        // {"move_fingertip", eMotionType_FINGERTIP_MOVING}
};

AllegroNodeGrasp::AllegroNodeGrasp()
        : AllegroNode() {

  initController(whichHand);

  joint_cmd_sub = nh.subscribe(
          DESIRED_STATE_TOPIC, 3, &AllegroNodeGrasp::setJointCallback, this);
  lib_cmd_sub = nh.subscribe(
          LIB_CMD_TOPIC, 1, &AllegroNodeGrasp::libCmdCallback, this);

  envelop_torque_sub = nh.subscribe(
          ENVELOP_TORQUE_TOPIC, 1, &AllegroNodeGrasp::envelopTorqueCallback,
          this);
}

AllegroNodeGrasp::~AllegroNodeGrasp() {
  delete pBHand;
}

void AllegroNodeGrasp::libCmdCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("CTRL: Heard: [%s]", msg->data.c_str());
  const std::string lib_cmd = msg->data;

  // Main behavior: apply the grasp directly from the map. Secondary behaviors can still be handled
  // normally (case-by-case basis), note these should *not* be in the map.
  auto itr = bhand_grasps.find(msg->data);
  if (itr != bhand_grasps.end()) {
    pBHand->SetMotionType(itr->second);
    ROS_INFO("motion type = %d", itr->second);
  } else if (lib_cmd.compare("rock") == 0) {
    pBHand->SetJointDesiredPosition(rock);
    pBHand->SetMotionType(eMotionType_JOINT_PD);
  } else if (lib_cmd.compare("paper") == 0) {
    pBHand->SetJointDesiredPosition(paper);
    pBHand->SetMotionType(eMotionType_JOINT_PD);
  } else if (lib_cmd.compare("scissors") == 0) {
    pBHand->SetJointDesiredPosition(scissors);
    pBHand->SetMotionType(eMotionType_JOINT_PD);
  } else if (lib_cmd.compare("bird") == 0) {
    pBHand->SetJointDesiredPosition(bird);
    pBHand->SetMotionType(eMotionType_JOINT_PD);
  } else if (lib_cmd.compare("aloha") == 0) {
    pBHand->SetJointDesiredPosition(aloha);
    pBHand->SetMotionType(eMotionType_JOINT_PD);
  } else if (lib_cmd.compare("horns") == 0) {
    pBHand->SetJointDesiredPosition(horns);
    pBHand->SetMotionType(eMotionType_JOINT_PD);
  } else if (lib_cmd.compare("one") == 0) {
    pBHand->SetJointDesiredPosition(one);
    pBHand->SetMotionType(eMotionType_JOINT_PD);
  } else if (lib_cmd.compare("two") == 0) {
    pBHand->SetJointDesiredPosition(two);
    pBHand->SetMotionType(eMotionType_JOINT_PD);
  } else if (lib_cmd.compare("three") == 0) {
    pBHand->SetJointDesiredPosition(three);
    pBHand->SetMotionType(eMotionType_JOINT_PD);
  } else if (lib_cmd.compare("start") == 0) {
    pBHand->SetJointDesiredPosition(start);
    pBHand->SetMotionType(eMotionType_JOINT_PD);
  } else if (lib_cmd.compare("pdControl") == 0) {
    // Desired position only necessary if in PD Control mode
    pBHand->SetJointDesiredPosition(desired_position);
    pBHand->SetMotionType(eMotionType_JOINT_PD);
  } else if (lib_cmd.compare("save") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position[i] = current_position[i];
  } else {
    ROS_WARN("Unknown commanded grasp: %s.", lib_cmd.c_str());
  }
}

// Called when a desired joint position message is received
void AllegroNodeGrasp::setJointCallback(const sensor_msgs::JointState &msg) {
  mutex->lock();

  for (int i = 0; i < DOF_JOINTS; i++)
    desired_position[i] = msg.position[i];
  mutex->unlock();

  pBHand->SetJointDesiredPosition(desired_position);
  pBHand->SetMotionType(eMotionType_JOINT_PD);
}

// The grasp controller can set the desired envelop grasp torque by listening to
// Float32 messages on ENVELOP_TORQUE_TOPIC ("allegroHand/envelop_torque").
void AllegroNodeGrasp::envelopTorqueCallback(const std_msgs::Float32 &msg) {
  const double torque = msg.data;
  ROS_INFO("Setting envelop torque to %.3f.", torque);
  pBHand->SetEnvelopTorqueScalar(torque);
}

void AllegroNodeGrasp::computeDesiredTorque() {
  // compute control torque using Bhand library
  pBHand->SetJointPosition(current_position_filtered);

  // BHand lib control updated with time stamp
  pBHand->UpdateControl((double) frame * ALLEGRO_CONTROL_TIME_INTERVAL);

  // Necessary torque obtained from Bhand lib
  pBHand->GetJointTorque(desired_torque);

  //ROS_INFO("desired torque = %.3f %.3f %.3f %.3f", desired_torque[0], desired_torque[1], desired_torque[2], desired_torque[3]);
}

void AllegroNodeGrasp::initController(const std::string &whichHand) {
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
  printf("        Custom (BHand) Method        \n");
  printf("-------------------------------------\n");
  printf("         Every command works.        \n");
  printf("*************************************\n");
}

void AllegroNodeGrasp::doIt(bool polling) {
  if (polling) {
    ROS_INFO("Polling = true.");
    while (ros::ok()) {
      updateController();
      ros::spinOnce();
    }
  } else {
    ROS_INFO("Polling = false.");

    // Timer callback (not recommended).
    ros::Timer timer = startTimerCallback();
    ros::spin();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "allegro_hand_core_grasp");
  AllegroNodeGrasp grasping;

  bool polling = false;
  if (argv[1] == std::string("true")) {
    polling = true;
  }
  ROS_INFO("Start controller with polling = %d", polling);

  grasping.doIt(polling);
}
