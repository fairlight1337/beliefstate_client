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


#include <beliefstate_client/BeliefstateClient.h>


namespace beliefstate_client {
  BeliefstateClient::BeliefstateClient(std::string strSource) {
    this->init(0, NULL, strSource);
  }

  BeliefstateClient::BeliefstateClient(int argc, char** argv, std::string strSource) {
    this->init(argc, argv, strSource);
  }

  BeliefstateClient::~BeliefstateClient() {
  }

  void BeliefstateClient::init(int argc, char** argv, std::string strSource, std::string strServer) {
    if(strServer == "") {
      strServer = "/beliefstate_ros";
    }
    
    m_strServer = strServer;
    this->setSource(strSource);
    
    if(!ros::isInitialized()) {
      ros::init(argc, argv, strSource);
    }
    
    m_nhHandle = new ros::NodeHandle("~");
    
    m_sclService = m_nhHandle->serviceClient<designator_integration_msgs::DesignatorCommunication>(m_strServer + "/operate");
  }
  
  std::list<CDesignator*> BeliefstateClient::callService(CDesignator* desigContent) {
    designator_integration_msgs::DesignatorCommunication dcComm;
    dcComm.request.request.designator = desigContent->serializeToMessage();
    
    std::list<CDesignator*> lstReturnDesigs;
    if(m_sclService.call(dcComm)) {
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
    
    desigRequest->setValue(string("_cb_type"), "begin");
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
      desigRequest->setValue("_time-start", nTimeStamp);
    }
    
    std::list<CDesignator*> lstDesigs = this->callService(desigRequest);
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

  void BeliefstateClient::endContext(int nID, bool bSuccess, int nTimeStamp) {
    CDesignator* desigRequest = new CDesignator();
    desigRequest->setType(ACTION);
    
    desigRequest->setValue(string("_cb_type"), "end");
    desigRequest->setValue(string("_id"), nID);
    desigRequest->setValue(string("_success"), (bSuccess ? 1 : 0));
    desigRequest->setValue(string("_source"), m_strSource);
    
    if(nTimeStamp > -1) {
      desigRequest->setValue("_time-end", nTimeStamp);
    }
    
    list<CDesignator*> lstDesigs = this->callService(desigRequest);
    
    for(CDesignator* cdDesig : lstDesigs) {
      delete cdDesig;
    }
    
    delete desigRequest;
  }

  std::list<CDesignator*> BeliefstateClient::alterContext(CDesignator* desigRequest) {
    desigRequest->setValue(string("_cb_type"), "alter");
    desigRequest->setValue(string("_type"), "alter");
    desigRequest->setValue(string("_source"), m_strSource);
    
    return this->callService(desigRequest);
  }
  
  void BeliefstateClient::discreteEvent(std::string strEventName, std::string strClassNamespace, std::string strClass, bool bSuccess, int nTimeStamp) {
    int nID = this->startContext(strEventName, strClassNamespace, strClass, nTimeStamp);
    this->endContext(nID, bSuccess, nTimeStamp);
  }

  void BeliefstateClient::addDesignator(CDesignator* cdAdd, std::string strAnnotation) {
    std::string strDesigType = "";
  
    switch(cdAdd->type()) {
    case OBJECT: {
      strDesigType = "OBJECT";
    } break;
    
    case LOCATION: {
      strDesigType = "LOCATION";
    } break;
    
    default:
    case ACTION: {
      strDesigType = "ACTION";
    } break;
    }
  
    std::stringstream sts;
    long lAddress = (long)cdAdd;
    sts << lAddress;
  
    CDesignator* cdSend = new CDesignator(ACTION);
    cdSend->addChild("description", LIST, cdAdd->children());
    cdSend->setValue("command", "add-designator");
    cdSend->setValue("type", strDesigType);
    cdSend->setValue("annotation", strAnnotation);
    cdSend->setValue("memory-address", sts.str());
  
    std::list<CDesignator*> lstResultDesignators = this->alterContext(cdSend);
  
    for(CDesignator* cdDelete : lstResultDesignators) {
      delete cdDelete;
    }
  
    delete cdSend;
  }

  void BeliefstateClient::annotateParameter(std::string strKey, std::string strValue) {
    CDesignator* cdAnnotate = new CDesignator(OBJECT);
    cdAnnotate->setValue(strKey, strValue);
  
    this->addDesignator(cdAnnotate, "parameter-annotation");
  
    delete cdAnnotate;
  }

  void BeliefstateClient::annotateParameter(std::string strKey, float fValue) {
    CDesignator* cdAnnotate = new CDesignator(OBJECT);
    cdAnnotate->setValue(strKey, fValue);
  
    this->addDesignator(cdAnnotate, "parameter-annotation");
  
    delete cdAnnotate;
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
    
    this->alterContext(desigRequest);
    
    desigRequest->setValue(std::string("format"), "dot");
    desigRequest->setValue(std::string("filename"), strFilename + ".dot");
    
    this->alterContext(desigRequest);
    
    delete desigRequest;
  }
  
  void BeliefstateClient::addObject(Object* objAdd, std::string strProperty) {
    std::stringstream sts;
    long lAddress = (long)objAdd;
    sts << lAddress;
    
    CDesignator* cdSend = new CDesignator(ACTION);
    cdSend->addChild("description", LIST, objAdd->children());
    cdSend->setValue("command", "add-object");
    cdSend->setValue("type", "OBJECT");
    cdSend->setValue("memory-address", sts.str());
    
    if(strProperty != "") {
      cdSend->setValue("property", strProperty);
    }
    
    if(objAdd->className() != "") {
      cdSend->setValue("class", objAdd->className());
    }
    
    if(objAdd->classNamespace() != "") {
      cdSend->setValue("classnamespace", objAdd->classNamespace());
    }
    
    std::list<CDesignator*> lstResultDesignators = this->alterContext(cdSend);
    
    for(CDesignator* cdDelete : lstResultDesignators) {
      delete cdDelete;
    }
    
    delete cdSend;
  }
}
