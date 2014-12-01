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
#include <designators/Designator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>

// Private
#include <beliefstate_client/Object.h>


namespace beliefstate_client {
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
    /*! \brief ROS service for communication with the beliefstate
        software */
    ros::ServiceClient m_sclService;
    
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
    
    void init(int argc, char** argv, std::string strSource, std::string strServer = "");
    
    void setSource(std::string strSource);
    std::string source();
    
    std::list<designator_integration::Designator*> callService(designator_integration::Designator* desigContent, int nRelativeContextID = -1);
    
    int startContext(std::string strContextName, int nRelativeToID = -1, double dTimeStamp = -1);
    int startContext(std::string strContextName, int nRelativeToID = -1, std::string strClassNamespace = "", std::string strClass = "", double dTimeStamp = -1);
    void endContext(int nID, bool bSuccess = true, double dTimeStamp = -1, bool bIsRelativeContextID = false);
    std::list<designator_integration::Designator*> alterContext(designator_integration::Designator* desigAlter, int nContextID = -1);
    
    void discreteEvent(std::string strEventName, int nToID = -1, std::string strClassNamespace = "", std::string strClass = "", bool bSuccess = true, double dTimeStamp = -1, bool bIsRelativeContextID = false);
    void addObject(Object* objAdd, std::string strProperty = "", int nToID = -1);
    
    void addDesignator(designator_integration::Designator* cdAdd, std::string strAnnotation = "", int nToID = -1);
    void annotateParameter(std::string strKey, std::string strValue, int nToID = -1);
    void annotateParameter(std::string strKey, float fValue, int nToID = -1);
    
    void exportFiles(std::string strFilename);
    void registerOWLNamespace(std::string strShortcut, std::string strIRI);
    
    void setMetaDataField(std::string strField, std::string strValue);
  };
}


#endif /* __BELIEFSTATE_CLIENT_H__ */
