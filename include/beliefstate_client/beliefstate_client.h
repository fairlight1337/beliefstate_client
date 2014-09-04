#ifndef __BELIEFSTATE_CLIENT_H__
#define __BELIEFSTATE_CLIENT_H__


// System
#include <iostream>
#include <string>

// ROS
#include <ros/ros.h>

// Designators
#include <designators/CDesignator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>

using namespace std;


class BeliefstateClient {
 private:
  ros::NodeHandle* m_nhHandle;
  string m_strSource;
  string m_strServer;
  ros::ServiceClient m_sclBeginContextService;
  ros::ServiceClient m_sclEndContextService;
  ros::ServiceClient m_sclAlterContextService;
  
 public:
  BeliefstateClient(int argc, char** argv, string strSource = "");
  ~BeliefstateClient();
  
  void setSource(string strSource);
  string source();
  
  list<CDesignator*> callService(ros::ServiceClient sclServiceClient, CDesignator* desigContent);
  
  int startContext(string strContextName, int nTimeStamp = -1);
  void endContext(int nContextID, bool bSuccess = true, int nTimeStamp = -1);
  list<CDesignator*> alterContext(CDesignator* desigAlter);
  
  void exportFiles(string strFilename);
};


#endif /* __BELIEFSTATE_CLIENT_H__ */
