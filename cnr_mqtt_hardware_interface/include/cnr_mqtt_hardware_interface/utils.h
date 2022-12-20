#include <cnr_mqtt_hardware_interface/cnr_mqtt_robot_hw.h>

static const char* DEFAULT      = "\033[0m";
static const char* RESET        = "\033[0m";
static const char* BLACK        = "\033[30m";
static const char* RED          = "\033[31m";
static const char* GREEN        = "\033[32m";
static const char* YELLOW       = "\033[33m";
static const char* BLUE         = "\033[34m";
static const char* MAGENTA      = "\033[35m";
static const char* CYAN         = "\033[36m";
static const char* WHITE        = "\033[37m";
static const char* BOLDBLACK    = "\033[1m\033[30m";
static const char* BOLDRED      = "\033[1m\033[31m";
static const char* BOLDGREEN    = "\033[1m\033[32m";
static const char* BOLDYELLOW   = "\033[1m\033[33m";
static const char* BOLDBLUE     = "\033[1m\033[34m";
static const char* BOLDMAGENTA  = "\033[1m\033[35m";
static const char* BOLDCYAN     = "\033[1m\033[36m";
static const char* BOLDWHITE    = "\033[1m\033[37m";




void mqtt_to_vector(const cnr::drapebot::MQTTDrapebotClientHw* client,std::vector<double> & ret)
{
  if(ret.size()!=7)
    ret.resize(7,0);
    
    ret.at(1) = client->mqtt_msg_dec_->J1;
    ret.at(2) = client->mqtt_msg_dec_->J2;
    ret.at(3) = client->mqtt_msg_dec_->J3;
    ret.at(4) = client->mqtt_msg_dec_->J4;
    ret.at(5) = client->mqtt_msg_dec_->J5;
    ret.at(6) = client->mqtt_msg_dec_->J6;
    ret.at(0) = client->mqtt_msg_dec_->E0;
}

void mqtt_msg_to_vector(const cnr::drapebot::drapebot_msg_hw msg,std::vector<double>& ret)
{
  if(ret.size()!=7)
    ret.resize(7,0);
  
    ret.at(1) = msg.J1;
    ret.at(2) = msg.J2;
    ret.at(3) = msg.J3;
    ret.at(4) = msg.J4;
    ret.at(5) = msg.J5;
    ret.at(6) = msg.J6;
    ret.at(0) = msg.E0;
}


void print_last_msg(cnr_logger::TraceLogger& logger, const cnr::drapebot::drapebot_msg_hw last_msg)
{    
    CNR_INFO_THROTTLE(logger,2.0,GREEN<<"last fb count "<< last_msg.count);
    CNR_INFO_THROTTLE(logger,2.0,GREEN<<"last fb J1 "<< last_msg.J1);
    CNR_INFO_THROTTLE(logger,2.0,GREEN<<"last fb J2 "<< last_msg.J2);
    CNR_INFO_THROTTLE(logger,2.0,GREEN<<"last fb J3 "<< last_msg.J3);
    CNR_INFO_THROTTLE(logger,2.0,GREEN<<"last fb J4 "<< last_msg.J4);
    CNR_INFO_THROTTLE(logger,2.0,GREEN<<"last fb J5 "<< last_msg.J5);
    CNR_INFO_THROTTLE(logger,2.0,GREEN<<"last fb J6 "<< last_msg.J6);
    CNR_INFO_THROTTLE(logger,2.0,GREEN<<"last fb E0 "<< last_msg.E0);
}


void vector_to_mqtt(const std::vector<double>& v, cnr::drapebot::MQTTDrapebotClientHw* client)
{
  assert(v.size()==7);
  
  client->mqtt_msg_dec_->J1 = v.at(0);
  client->mqtt_msg_dec_->J2 = v.at(1);
  client->mqtt_msg_dec_->J3 = v.at(2);
  client->mqtt_msg_dec_->J4 = v.at(3);
  client->mqtt_msg_dec_->J5 = v.at(4);
  client->mqtt_msg_dec_->J6 = v.at(5);
  client->mqtt_msg_dec_->E0 = v.at(6);
}
  
void vector_to_mqtt_msg(const std::vector<double>& v, cnr::drapebot::drapebot_msg_hw& m)
{
  assert(v.size()==7);
  
  m.J1 = v.at(1);
  m.J2 = v.at(2);
  m.J3 = v.at(3);
  m.J4 = v.at(4);
  m.J5 = v.at(5);
  m.J6 = v.at(6);
  m.E0 = v.at(0);
}

void print_vector(cnr_logger::TraceLogger& logger, const std::string& msg, const std::vector<double>& v, const char* color = "\033[0m" )
{
  assert(v.size()==7);
  for(size_t i=0;i< v.size();i++)
  {
    CNR_INFO(logger, color << msg << " " << (i==0u? "E" : "J") << i << ": "  << v.at(i));
  }
}
void print_vector_throttle(cnr_logger::TraceLogger& logger, const std::string& msg, const std::vector<double>& v ,double throttle = 1.0, const char* color = "\033[0m")
{
  assert(v.size()==7);
  for(size_t i=0;i< v.size();i++)
  {
    CNR_INFO_THROTTLE(logger, throttle, color << msg << " J1: "  << v.at(1));
    CNR_INFO_THROTTLE(logger, throttle, color << msg << " J2: "  << v.at(2));
    CNR_INFO_THROTTLE(logger, throttle, color << msg << " J3: "  << v.at(3));
    CNR_INFO_THROTTLE(logger, throttle, color << msg << " J4: "  << v.at(4));
    CNR_INFO_THROTTLE(logger, throttle, color << msg << " J5: "  << v.at(5));
    CNR_INFO_THROTTLE(logger, throttle, color << msg << " J6: "  << v.at(6));
    CNR_INFO_THROTTLE(logger, throttle, color << msg << " E0: "  << v.at(0));
  }
}

void print_message_struct_throttle(cnr_logger::TraceLogger& logger, const std::string& msg, const cnr::drapebot::drapebot_msg_hw& m, double throttle = 1.0)
{
  CNR_INFO_THROTTLE(logger,throttle , msg << " J1 : " << m.J1);
  CNR_INFO_THROTTLE(logger,throttle , msg << " J2 : " << m.J2);
  CNR_INFO_THROTTLE(logger,throttle , msg << " J3 : " << m.J3);
  CNR_INFO_THROTTLE(logger,throttle , msg << " J4 : " << m.J4);
  CNR_INFO_THROTTLE(logger,throttle , msg << " J5 : " << m.J5);
  CNR_INFO_THROTTLE(logger,throttle , msg << " J6 : " << m.J6);
  CNR_INFO_THROTTLE(logger,throttle , msg << " E0 : " << m.E0);
}

