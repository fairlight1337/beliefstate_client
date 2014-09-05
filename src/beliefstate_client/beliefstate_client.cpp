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


#include <beliefstate_client/beliefstate_client.h>


BeliefstateClient::BeliefstateClient(std::string strSource) {
  this->init(0, NULL, strSource);
}

BeliefstateClient::BeliefstateClient(int argc, char** argv, std::string strSource) {
  this->init(argc, argv, strSource);
}

BeliefstateClient::~BeliefstateClient() {
}

void BeliefstateClient::init(int argc, char** argv, std::string strSource) {
  m_strServer = "/beliefstate_ros";
  this->setSource(strSource);
  
  if(!ros::ok() && argc > 0 && argv) {
    ros::init(argc, argv, strSource);
    m_nhHandle = new ros::NodeHandle("~");
  } else {
    ros::NodeHandle nhHandle("~");
    m_nhHandle = &nhHandle;
  }
  
  m_sclBeginContextService = m_nhHandle->serviceClient<designator_integration_msgs::DesignatorCommunication>(m_strServer + "/begin_context");
  m_sclEndContextService = m_nhHandle->serviceClient<designator_integration_msgs::DesignatorCommunication>(m_strServer + "/end_context");
  m_sclAlterContextService = m_nhHandle->serviceClient<designator_integration_msgs::DesignatorCommunication>(m_strServer + "/alter_context");
}

std::list<CDesignator*> BeliefstateClient::callService(ros::ServiceClient sclServiceClient, CDesignator* desigContent) {
  designator_integration_msgs::DesignatorCommunication dcComm;
  dcComm.request.request.designator = desigContent->serializeToMessage();
  
  std::list<CDesignator*> lstReturnDesigs;
  if(sclServiceClient.call(dcComm)) {
    for(designator_integration_msgs::Designator msgDesig : dcComm.response.response.designators) {
      lstReturnDesigs.push_back(new CDesignator(msgDesig));
    }
  }
  
  return lstReturnDesigs;
}

void BeliefstateClient::setSource(std::string strSource) {
  m_strSource = strSource;
}

std::string BeliefstateClient::source() {
  return m_strSource;
}

int BeliefstateClient::startContext(std::string strContextName, int nTimeStamp) {
  return this->startContext(strContextName, "", "", nTimeStamp);
}

int BeliefstateClient::startContext(std::string strContextName, std::string strClassNamespace, std::string strClass, int nTimeStamp) {
  CDesignator* desigRequest = new CDesignator();
  desigRequest->setType(ACTION);
  desigRequest->setValue(string("_name"), strContextName);
  desigRequest->setValue(string("_source"), m_strSource);
  desigRequest->setValue(string("_detail-level"), 1);
  
  if(strClass != "") {
    desigRequest->setValue(string("_class"), strClass);
    
    if(strClassNamespace != "") {
      desigRequest->setValue(string("_classnamespace"), strClassNamespace);
    }
  }
  
  if(nTimeStamp > -1) {
    desigRequest->setValue("time-start", nTimeStamp);
  }
  
  std::list<CDesignator*> lstDesigs = this->callService(m_sclBeginContextService, desigRequest);
  delete desigRequest;
  
  int nID = -1;
  if(lstDesigs.size() == 1) {
    CDesignator* desigResponse = lstDesigs.front();
    nID = (int)desigResponse->floatValue("_id");
  }
  
  for(CDesignator* cdDesig : lstDesigs) {
    delete cdDesig;
  }
  
  return nID;
}

void BeliefstateClient::endContext(int nContextID, bool bSuccess, int nTimeStamp) {
  CDesignator* desigRequest = new CDesignator();
  desigRequest->setType(ACTION);
  
  desigRequest->setValue(string("_id"), nContextID);
  desigRequest->setValue(string("_success"), (bSuccess ? 1 : 0));
  desigRequest->setValue(string("_source"), m_strSource);
  
  if(nTimeStamp > -1) {
    desigRequest->setValue("time-end", nTimeStamp);
  }
  
  list<CDesignator*> lstDesigs = this->callService(m_sclEndContextService, desigRequest);
  
  for(CDesignator* cdDesig : lstDesigs) {
    delete cdDesig;
  }
  
  delete desigRequest;
}

std::list<CDesignator*> BeliefstateClient::alterContext(CDesignator* desigAlter) {
  desigAlter->setValue(string("_source"), m_strSource);
  
  return this->callService(m_sclAlterContextService, desigAlter);
}

void BeliefstateClient::discreteEvent(std::string strEventName, std::string strClassNamespace, std::string strClass, bool bSuccess, int nTimeStamp) {
  int nCtxID = this->startContext(strEventName, strClassNamespace, strClass, nTimeStamp);
  this->endContext(nCtxID, bSuccess, nTimeStamp);
}

void BeliefstateClient::exportFiles(std::string strFilename) {
  CDesignator* desigRequest = new CDesignator();
  desigRequest->setType(ACTION);
  
  desigRequest->setValue(std::string("command"), "export-planlog");
  desigRequest->setValue(std::string("format"), "owl");
  desigRequest->setValue(std::string("filename"), strFilename + ".owl");
  desigRequest->setValue(std::string("show-successes"), 1);
  desigRequest->setValue(std::string("show-fails"), 1);
  desigRequest->setValue(std::string("max-detail-level"), 99);
  
  this->callService(m_sclAlterContextService, desigRequest);
  
  desigRequest->setValue(std::string("format"), "dot");
  desigRequest->setValue(std::string("filename"), strFilename + ".dot");
  
  this->callService(m_sclAlterContextService, desigRequest);
  
  delete desigRequest;
}
