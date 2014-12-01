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
  
  std::list<designator_integration::Designator*> BeliefstateClient::callService(designator_integration::Designator* desigContent, int nRelativeContextID) {
    if(nRelativeContextID > -1) {
      desigContent->setValue(std::string("_relative_context_id"), nRelativeContextID);
    }
    
    designator_integration_msgs::DesignatorCommunication dcComm;
    dcComm.request.request.designator = desigContent->serializeToMessage();
    
    std::list<designator_integration::Designator*> lstReturnDesigs;
    if(m_sclService.call(dcComm)) {
      for(designator_integration_msgs::Designator msgDesig : dcComm.response.response.designators) {
	lstReturnDesigs.push_back(new designator_integration::Designator(msgDesig));
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
  
  int BeliefstateClient::startContext(std::string strContextName, int nRelativeToID, double dTimeStamp) {
    return this->startContext(strContextName, nRelativeToID, "", "", dTimeStamp);
  }

  int BeliefstateClient::startContext(std::string strContextName, int nRelativeToID, std::string strClassNamespace, std::string strClass, double dTimeStamp) {
    designator_integration::Designator* desigRequest = new designator_integration::Designator();
    desigRequest->setType(designator_integration::Designator::DesignatorType::ACTION);
    
    desigRequest->setValue(std::string("_cb_type"), "begin");
    desigRequest->setValue(std::string("_name"), strContextName);
    desigRequest->setValue(std::string("_source"), m_strSource);
    desigRequest->setValue(std::string("_detail-level"), 1);
    
    if(strClass != "") {
      desigRequest->setValue(std::string("_class"), strClass);
    
      if(strClassNamespace != "") {
	desigRequest->setValue(std::string("_classnamespace"), strClassNamespace);
      }
    }
    
    if(dTimeStamp > -1) {
      desigRequest->setValue("_time-start", dTimeStamp);
    }
    
    std::list<designator_integration::Designator*> lstDesigs = this->callService(desigRequest, nRelativeToID);
    delete desigRequest;
    
    int nID = -1;
    if(lstDesigs.size() == 1) {
      designator_integration::Designator* desigResponse = lstDesigs.front();
      nID = (int)desigResponse->floatValue("_id");
    }
    
    for(designator_integration::Designator* cdDesig : lstDesigs) {
      delete cdDesig;
    }
    
    return nID;
  }

  void BeliefstateClient::endContext(int nID, bool bSuccess, double dTimeStamp, bool bIsRelativeContextID) {
    designator_integration::Designator* desigRequest = new designator_integration::Designator();
    desigRequest->setType(designator_integration::Designator::DesignatorType::ACTION);
    
    desigRequest->setValue(std::string("_cb_type"), "end");
    desigRequest->setValue(std::string("_id"), nID);
    desigRequest->setValue(std::string("_success"), (bSuccess ? 1 : 0));
    desigRequest->setValue(std::string("_source"), m_strSource);
    
    if(dTimeStamp > -1) {
      desigRequest->setValue("_time-end", dTimeStamp);
    }
    
    std::list<designator_integration::Designator*> lstDesigs;
    if(bIsRelativeContextID) {
      lstDesigs = this->callService(desigRequest, nID);
    } else {
      lstDesigs = this->callService(desigRequest);
    }
    
    for(designator_integration::Designator* cdDesig : lstDesigs) {
      delete cdDesig;
    }
    
    delete desigRequest;
  }

  std::list<designator_integration::Designator*> BeliefstateClient::alterContext(designator_integration::Designator* desigRequest, int nContextID) {
    desigRequest->setValue(std::string("_cb_type"), "alter");
    desigRequest->setValue(std::string("_type"), "alter");
    desigRequest->setValue(std::string("_source"), m_strSource);
    
    return this->callService(desigRequest, nContextID);
  }
  
  void BeliefstateClient::discreteEvent(std::string strEventName, int nToID, std::string strClassNamespace, std::string strClass, bool bSuccess, double dTimeStamp, bool bIsRelativeContextID) {
    int nID = this->startContext(strEventName, nToID, strClassNamespace, strClass, dTimeStamp);
    this->endContext(nID, bSuccess, dTimeStamp, bIsRelativeContextID);
  }
  
  void BeliefstateClient::addDesignator(designator_integration::Designator* cdAdd, std::string strAnnotation, int nToID) {
    std::string strDesigType = "";
    
    switch(cdAdd->type()) {
    case designator_integration::Designator::DesignatorType::OBJECT: {
      strDesigType = "OBJECT";
    } break;
    
    case designator_integration::Designator::DesignatorType::LOCATION: {
      strDesigType = "LOCATION";
    } break;
    
    default:
    case designator_integration::Designator::DesignatorType::ACTION: {
      strDesigType = "ACTION";
    } break;
    }
    
    std::stringstream sts;
    long lAddress = (long)cdAdd;
    sts << lAddress;
  
    designator_integration::Designator* cdSend = new designator_integration::Designator(designator_integration::Designator::DesignatorType::ACTION);
    cdSend->addChild("description", designator_integration::KeyValuePair::ValueType::LIST, cdAdd->children());
    cdSend->setValue("command", "add-designator");
    cdSend->setValue("type", strDesigType);
    cdSend->setValue("annotation", strAnnotation);
    cdSend->setValue("memory-address", sts.str());
    
    std::list<designator_integration::Designator*> lstResultDesignators = this->alterContext(cdSend, nToID);
    
    for(designator_integration::Designator* cdDelete : lstResultDesignators) {
      delete cdDelete;
    }
    
    delete cdSend;
  }
  
  void BeliefstateClient::annotateParameter(std::string strKey, std::string strValue, int nToID) {
    designator_integration::Designator* cdAnnotate = new designator_integration::Designator(designator_integration::Designator::DesignatorType::OBJECT);
    cdAnnotate->setValue(strKey, strValue);
    
    this->addDesignator(cdAnnotate, "parameter-annotation", nToID);
    
    delete cdAnnotate;
  }

  void BeliefstateClient::annotateParameter(std::string strKey, float fValue, int nToID) {
    designator_integration::Designator* cdAnnotate = new designator_integration::Designator(designator_integration::Designator::DesignatorType::OBJECT);
    cdAnnotate->setValue(strKey, fValue);
    
    this->addDesignator(cdAnnotate, "parameter-annotation", nToID);
    
    delete cdAnnotate;
  }
  
  void BeliefstateClient::exportFiles(std::string strFilename) {
    designator_integration::Designator* desigRequest = new designator_integration::Designator();
    desigRequest->setType(designator_integration::Designator::DesignatorType::ACTION);
    
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
  
  void BeliefstateClient::registerOWLNamespace(std::string strShortcut, std::string strIRI) {
    designator_integration::Designator* desigRequest = new designator_integration::Designator();
    desigRequest->setType(designator_integration::Designator::DesignatorType::ACTION);
    
    desigRequest->setValue(std::string("command"), "register-owl-namespace");
    desigRequest->setValue(std::string("shortcut"), strShortcut);
    desigRequest->setValue(std::string("iri"), strIRI);
    
    this->alterContext(desigRequest);
    
    delete desigRequest;
  }
  
  void BeliefstateClient::addObject(Object* objAdd, std::string strProperty, int nToID) {
    std::stringstream sts;
    long lAddress = (long)objAdd;
    sts << lAddress;
    
    designator_integration::Designator* cdSend = new designator_integration::Designator(designator_integration::Designator::DesignatorType::ACTION);
    cdSend->addChild("description", designator_integration::KeyValuePair::ValueType::LIST, objAdd->children());
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
    
    this->sendDesignator(cdSend);
  }
  
  void BeliefstateClient::sendDesignator(designator_integration::Designator* cdSend, bool bDelete) {
    std::list<designator_integration::Designator*> lstResultDesignators = this->alterContext(cdSend);
    
    if(bDelete) {
      for(designator_integration::Designator* cdDelete : lstResultDesignators) {
	delete cdDelete;
      }
    
      delete cdSend;
    }
  }
  
  void BeliefstateClient::setMetaDataField(std::string strField, std::string strValue) {
    designator_integration::Designator* cdSend = new designator_integration::Designator(designator_integration::Designator::DesignatorType::ACTION);
    cdSend->setValue("command", "set-experiment-meta-data");
    cdSend->setValue("field", strField);
    cdSend->setValue("value", strValue);
    
    this->sendDesignator(cdSend);
  }
  
  void BeliefstateClient::startNewExperiment() {
    designator_integration::Designator* cdSend = new designator_integration::Designator(designator_integration::Designator::DesignatorType::ACTION);
    cdSend->setValue("command", "start-new-experiment");
    
    this->sendDesignator(cdSend);
  }
}
