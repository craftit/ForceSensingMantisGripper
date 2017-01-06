
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "control_msgs/GripperCommandAction.h"
#include "actionlib/server/simple_action_server.h"
#include "serial/serial.h"

#include <sstream>

#include <thread>
#include <vector>
#include <functional>
#include <assert.h>
#include <mutex>

class SerialComsC
{
public:
  SerialComsC(const char *portAddr);

  //! default
  SerialComsC();

  bool Open(const char *portAddr);

  //! Accept a byte
  void AcceptByte(uint8_t sendByte);

  //! Process received packet.
  void ProcessPacket();

  //! Send packet
  void SendPacket(uint8_t *data,int len);

  //! Send a move command
  void SendMove(int servoId,int pos);

  //! Send a move command with an effort level.
  void SendMoveWithEffort(float pos,float effort);

  //! Send a move command
  void SendPing();

  //! Set the handler for a particular type of packet.
  void SetHandler(int packetId,const std::function<void (uint8_t *data,int )> &handler);

  volatile bool m_terminate = false;
  int m_state = 0;
  int m_checkSum = 0;
  int m_packetLen = 0;
  uint8_t m_data[64];
  int m_at = 0;
  const uint8_t m_charSTX = 0x02;
  const uint8_t m_charETX = 0x03;
  std::shared_ptr<serial::Serial> m_serialPort;

  // Packet structure.
  // x    STX
  // x    Len - Of data excluding STX,ETX and Checksum.
  // 0    Address
  // 1    Type
  // 2..n data.
  // n    2-CRC
  // n    ETX.

  bool RunRecieve();

  std::thread m_threadRecieve;

  std::mutex m_accessPacketHandler;
  std::mutex m_accessTx;

  std::vector<std::function<void (uint8_t *data,int )> > m_packetHandler;
};



SerialComsC::SerialComsC(const char *portAddr)
{
  Open(portAddr);
}

//! default
SerialComsC::SerialComsC()
{}

bool SerialComsC::Open(const char *portAddr)
{
  if(m_serialPort) {
    ROS_ERROR("Serial port already opened.");
    return false;
  }

  m_serialPort = std::make_shared<serial::Serial>(portAddr, 38400, serial::Timeout::simpleTimeout(1000));

  if(!m_serialPort->isOpen()) {
    std::cout << "Failed to open serial port. " << std::endl;
    return false;
  }

  m_threadRecieve = std::move(std::thread { [this]{ RunRecieve(); } });
  return true;
}


void SerialComsC::SetHandler(int packetId,const std::function<void (uint8_t *data,int )> &handler)
{
  std::lock_guard<std::mutex> lock(m_accessPacketHandler);
  assert(packetId < 256);

  while(m_packetHandler.size() <= packetId) {
    m_packetHandler.push_back(std::function<void (uint8_t *data,int )>());
  }
  m_packetHandler[packetId] = handler;
}

void SerialComsC::AcceptByte(uint8_t sendByte)
{
  switch(m_state)
  {
  case 0: // Wait for STX.
    if(sendByte == m_charSTX)
      m_state = 1;
    // Else remain in state 0.
    break;
  case 1: // Packet length.
    m_packetLen = sendByte;
    if(m_packetLen > 64) {
      m_state = 0;
      break;
    }

    m_at = 0;
    m_checkSum = 0x55 + m_packetLen;
    m_state = 2;
    break;
  case 2: // Data
    m_checkSum += sendByte;
    m_data[m_at] = sendByte;
    m_at++;
    if(m_at >= m_packetLen)
      m_state = 3;
    break;
  case 3: { // CRC 1
    uint8_t cs1 = (m_checkSum & 0xff);
    //ROS_DEBUG("Checksum1 : %d %d ",(int)  cs1 , (int) sendByte);
    if(cs1 != sendByte) {
      ROS_DEBUG("Checksum failed. ");
      if(sendByte == m_charSTX)
        m_state = 1;
      else
        m_state = 0;
      break;
    }

    m_state = 4;
  } break;
  case 4: { // CRC 1
    uint8_t cs2 = ((m_checkSum >> 8) & 0xff);
    //ROS_DEBUG("Checksum2 : %d %d ",(int) ((m_checkSum >> 8) & 0xff) , (int) sendByte);
    if(cs2 != sendByte) {
      ROS_DEBUG("Checksum failed. ");
      if(sendByte == m_charSTX)
        m_state = 1;
      else
        m_state = 0;
      break;
    }

    m_state = 5;
  } break;
  case 5: // ETX.
    if(sendByte == m_charETX) {
      //ROS_DEBUG("Got packet!");
      ProcessPacket();
    } else {
      ROS_DEBUG("Packet corrupted.");
    }
    m_state = 0;
    break;
  }
}

//! Process received packet.
void SerialComsC::ProcessPacket()
{
  std::string data;
  for(int i = 0;i < m_packetLen;i++) {
    data += std::to_string(m_data[i]) + " ";
  }
  ROS_DEBUG("Got packet [%d] %s ",m_packetLen,data.c_str());

  // m_data[0] //
  int packetId = m_data[0];
  switch(packetId)
  {
    case 1:
      break;
    case 4:
      ROS_ERROR("Received error code: %d  Arg:%d ",(int) m_data[1],(int) m_data[2]);
      return ;
      break;
    default: {
      std::lock_guard<std::mutex> lock(m_accessPacketHandler);
      if(packetId < m_packetHandler.size()) {
        const std::function<void (uint8_t *data,int )> &handler = m_packetHandler[packetId];
        if(handler) {
          handler(m_data,m_packetLen);
          return ;
        }
      }
      ROS_DEBUG("Don't know how to handle packet %d (Of %d) ",packetId,(int) m_packetHandler.size());
    } break;
#if 0
  case 2: { // ADC Data.
    int current = ((int) m_data[2])  + (((int) m_data[3]) << 8);
    int volt = ((int) m_data[4])  + (((int) m_data[5]) << 8);
    ROS_DEBUG("I:%4d V:%4d  Phase:%d  PWM:%d ",current,volt,(int) m_data[6],(int) m_data[7]);
  } break;
  default:
    ROS_DEBUG("Unexpected packet type %d ",(int) m_data[1]);
    break;
#endif
  }
}

bool SerialComsC::RunRecieve()
{
  ROS_DEBUG("Running receiver");
  assert(m_serialPort);
  while(!m_terminate) {
    uint8_t readByte = 0x55;
    // FIXME:- Read into a larger buffer.
    m_serialPort->read(&readByte,1);
    //ROS_DEBUG("Got byte %d ",(int) readByte);
    AcceptByte(readByte);
  }
  ROS_DEBUG("Done receiver");

  return true;
}

//! Send packet
void SerialComsC::SendPacket(uint8_t *buff,int len)
{
  assert(m_serialPort);
  std::lock_guard<std::mutex> lock(m_accessTx);

  std::vector<uint8_t> packet;
  packet.reserve(len + 6);


  packet.push_back(m_charSTX);
  int crc = len + 0x55;
  packet.push_back(len);
  for(int i =0;i < len;i++) {
    uint8_t data = buff[i];
    packet.push_back(data);
    crc += data;
  }
  packet.push_back(crc);
  packet.push_back(crc >> 8);
  packet.push_back(m_charETX);

  m_serialPort->write(&packet[0],packet.size());
}


//! Send a move command
void SerialComsC::SendMove(int servoId,int pos)
{
  uint8_t data[64];

  int at = 0;
  data[at++] = 3;
  data[at++] = servoId;
  data[at++] = (int) pos;

  SendPacket(data,at);
}

//! Send a move command
void SerialComsC::SendMoveWithEffort(float pos,float effort)
{
  uint8_t data[64];

  int at = 0;
  data[at++] = 6;
  data[at++] = 0; // SM_Position

  if(pos < 0)
    pos = 0;
  if(pos > 1.0)
    pos = 1.0;

  int xpos = 1024.0 - (pos * 1024.0 * 2.0 / 3.14159265359); // 0 to 45 degress in radians
  data[at++] = xpos;
  data[at++] = xpos >> 8;

  if(effort < 0)
    effort = 0;
  if(effort > 1.0)
    effort = 1.0;

  int xeffort = effort * 1024.0;
  data[at++] = xeffort;
  data[at++] = xeffort >> 8;

  // We don't use the velocity limit at the moment
  int xvelLimit = 1024;
  data[at++] = xvelLimit;
  data[at++] = xvelLimit >> 8;

  SendPacket(data,at);
}


//! Send a move command
void SerialComsC::SendPing()
{
  uint8_t data[64];

  int at = 0;
  data[at++] = 1;

  SendPacket(data,at);
}


void commandCallback(const control_msgs::GripperCommand::ConstPtr& msg)
{
  ROS_INFO("I heard: Position=%f MaxEffort =%f ", msg->position,msg->max_effort);

}

typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction> Server;

void ExecuteAction(const control_msgs::GripperCommandGoalConstPtr& goal, Server* as,SerialComsC *coms)
{
  ROS_INFO("Executing action.");

  control_msgs::GripperCommandFeedback msg;
  std::mutex done;
  done.lock();

  bool preempted = false;

  as->registerPreemptCallback([&preempted,&done]( ){ preempted = true; done.unlock(); });

  coms->SendMoveWithEffort(goal->command.position,goal->command.max_effort);

  coms->SetHandler(5,
                  [as,&msg,&done](uint8_t *data,int size) mutable
                  {
                    int16_t pos = (unsigned) data[1] + ((unsigned) data[2] << 8);
                    int16_t effort = (unsigned) data[3] + ((unsigned) data[4] << 8);
                    int32_t velocity = (unsigned) data[5] + ((unsigned) data[6] << 8) + ((unsigned) data[7] << 16) + ((unsigned) data[8] << 24);
                    bool atGoal = data[9];
                    bool stalled = data[10];

                    ROS_INFO("Pos:%d Effort:%d Velocity:%d  Goal:%d Stalled:%d ",pos,effort,(int) velocity,atGoal,stalled);

                    //control_msgs::GripperCommandFeedback msg;

                    msg.position = 1.0 - ((pos * 3.14159265359)/ (2 * 1024.0));
                    if(msg.position < 0) msg.position = 0;
                    if(msg.position > 1.0) msg.position = 1.0;
                    msg.effort = (float) (effort / 1024.0) - 0.5;
                    msg.stalled = stalled;
                    msg.reached_goal = atGoal;

                    as->publishFeedback(msg);

                    // Are we finished with action?
                    if(stalled || atGoal)
                      done.unlock();
                  });


  done.lock();

  control_msgs::GripperCommandResult resultMsg;
  resultMsg.position = msg.position;
  resultMsg.effort = msg.effort;
  resultMsg.stalled = msg.stalled;
  resultMsg.reached_goal = msg.reached_goal;

  if(preempted) {
    as->setPreempted(resultMsg);
    return ;
  }

  as->setSucceeded(resultMsg);
}



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "mantis_gripper");

  ros::NodeHandle n;
  ros::NodeHandle nPrivate("~");


  //ros::Subscriber sub = n.subscribe("gripper_command", 1000, commandCallback);

  ///dev/ttyACM0
  std::string devFilename = "/dev/ttyUSB0";
  nPrivate.getParam("devfilename",devFilename);

#if 1
  std::string gripperStatusTopic = "gripper_state";
  nPrivate.getParam("gripperStatusTopic",gripperStatusTopic);

  std::string gripperCommandTopic = "gripper_command";
  nPrivate.getParam("gripperCommandTopic",gripperCommandTopic);
#endif


  bool simMode = false;
  nPrivate.getParam("sim",simMode);

  ros::Rate loop_rate(10);

  ROS_DEBUG("Connecting");

  ros::Subscriber sub;
  SerialComsC coms;

  if(!simMode) {
    if(!coms.Open(devFilename.c_str())) {
      ROS_ERROR("Failed to open serial port '%s' ",devFilename.c_str());
      return 1;
    }
  }

#if 0

  Server server(n, "gripper", boost::bind(&ExecuteAction, _1, &server,&coms), false);



#else
  sub = n.subscribe<control_msgs::GripperCommand>(gripperCommandTopic, 100,
                                                                  [&coms,simMode](const control_msgs::GripperCommand::ConstPtr& msg)
                                                                  {
                                                                    if(!simMode)
                                                                      coms.SendMoveWithEffort(msg->position,msg->max_effort);
                                                                    //ROS_INFO("Move gripper to %f  with effort %f ",msg->position,msg->max_effort);
                                                                  }
                                                                  );

  ros::Publisher gripper_state = n.advertise<control_msgs::GripperCommandResult>(gripperStatusTopic, 100);

  coms.SetHandler(5,
                  [&gripper_state](uint8_t *data,int size)
                  {
                    int16_t pos = (unsigned) data[1] + ((unsigned) data[2] << 8);
                    int16_t effort = (unsigned) data[3] + ((unsigned) data[4] << 8);
                    int32_t velocity = (unsigned) data[5] + ((unsigned) data[6] << 8) + ((unsigned) data[7] << 16) + ((unsigned) data[8] << 24);
                    bool atGoal = data[9];
                    bool stalled = data[10];

                    ROS_INFO("Pos:%d Effort:%d Velocity:%d  Goal:%d Stalled:%d ",pos,effort,(int) velocity,atGoal,stalled);


                    control_msgs::GripperCommandResult msg;


                    msg.position = 1.0 - ((pos * 3.14159265359)/ (2 * 1024.0));
                    if(msg.position < 0) msg.position = 0;
                    if(msg.position > 1.0) msg.position = 1.0;
                    msg.effort = (float) (effort / 1024.0) - 0.5;
                    msg.stalled = stalled;
                    msg.reached_goal = atGoal;

                    gripper_state.publish(msg);

                  }
    );
    ros::spin();
#endif


  return 0;
}
