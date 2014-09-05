/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Jan Winkler, Institute for Artificial
 *  Intelligence, Universität Bremen.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Institute for Artificial Intelligence,
 *     Universität Bremen, nor the names of its contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Jan Winkler */


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


/*! \brief Client adapter for interfacing with the beliefstate ROS package */
class BeliefstateClient {
 private:
  /*! \brief The current instance's ROS node handle */
  ros::NodeHandle* m_nhHandle;
  /*! \brief Name string for identifying this client in the logging system
   
    This defaults to the client's ROS node name if not specified manually. */
  std::string m_strSource;
  /*! \brief The ROS node name of the beliefstate node to talk to
    
    This defaults to '/beliefstate_ros'. */
  std::string m_strServer;
  /*! \brief ROS service handle for starting log contexts */
  ros::ServiceClient m_sclBeginContextService;
  /*! \brief ROS service handle for ending log contexts */
  ros::ServiceClient m_sclEndContextService;
  /*! \brief ROS service handle for altering log contexts */
  ros::ServiceClient m_sclAlterContextService;
  
 public:
  /*! \brief Constructor not opening a new ROS node
   
    This constructor relies on an already existing ROS node within the
    linked program in which this class is used.
  
    \param strSource Identifier string for identification at the logging server node */
  BeliefstateClient(std::string strSource = "");
  /*! \brief Constructor opening a new ROS node
    
    This constructor starts a new ROS node for this instance. If a ROS
    node instance is already present, no new instance is started.
  
    \param argc The argc parameter as passed to the main() function
    \param argc The argv parameter as passed to the main() function
    \param strSource Identifier string for identification at the logging server node */
  BeliefstateClient(int argc, char** argv, std::string strSource = "");
  /*! \brief Destructor for the adapter class */
  ~BeliefstateClient();
  
  void init(int argc, char** argv, std::string strSource);

  void setSource(std::string strSource);
  std::string source();
  
  std::list<CDesignator*> callService(ros::ServiceClient sclServiceClient, CDesignator* desigContent);
  
  int startContext(std::string strContextName, int nTimeStamp = -1);
  int startContext(std::string strContextName, std::string strClassNamespace = "", std::string strClass = "", int nTimeStamp = -1);
  void endContext(int nContextID, bool bSuccess = true, int nTimeStamp = -1);
  list<CDesignator*> alterContext(CDesignator* desigAlter);
  
  void discreteEvent(std::string strEventName, std::string strClassNamespace = "", std::string strClass = "", bool bSuccess = true, int nTimeStamp = -1);
  
  void addDesignator(CDesignator* cdAdd, std::string strAnnotation = "");
  void annotateParameter(std::string strKey, std::string strValue);
  void annotateParameter(std::string strKey, float fValue);
  
  void exportFiles(std::string strFilename);
};


#endif /* __BELIEFSTATE_CLIENT_H__ */
