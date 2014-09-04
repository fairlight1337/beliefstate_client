#include <beliefstate_client/beliefstate_client.h>


BeliefstateClient::BeliefstateClient(int argc, char** argv, string strSource) {
  m_strServer = "/beliefstate_ros";
  this->setSource(strSource);
  
  if(!ros::ok()) {
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

BeliefstateClient::~BeliefstateClient() {
}

list<CDesignator*> BeliefstateClient::callService(ros::ServiceClient sclServiceClient, CDesignator* desigContent) {
  designator_integration_msgs::DesignatorCommunication dcComm;
  dcComm.request.request.designator = desigContent->serializeToMessage();
  
  list<CDesignator*> lstReturnDesigs;
  if(sclServiceClient.call(dcComm)) {
    vector<designator_integration_msgs::Designator> vecDesigResponses = dcComm.response.response.designators;
    
    for(vector<designator_integration_msgs::Designator>::iterator itDesig = vecDesigResponses.begin();
	itDesig != vecDesigResponses.end();
	itDesig++) {
      lstReturnDesigs.push_back(new CDesignator((*itDesig)));
    }
  }
  
  return lstReturnDesigs;
}

void BeliefstateClient::setSource(string strSource) {
  m_strSource = strSource;
}

string BeliefstateClient::source() {
  return m_strSource;
}

int BeliefstateClient::startContext(string strContextName) {
  CDesignator* desigRequest = new CDesignator();
  desigRequest->setType(ACTION);
  desigRequest->setValue(string("_name"), strContextName);
  desigRequest->setValue(string("_source"), m_strSource);
  desigRequest->setValue(string("_detail-level"), 1);
  
  list<CDesignator*> lstDesigs = this->callService(m_sclBeginContextService,
						   desigRequest);
  delete desigRequest;
  
  int nID = -1;
  if(lstDesigs.size() == 1) {
    CDesignator* desigResponse = lstDesigs.front();
    nID = (int)desigResponse->floatValue("_id");
  }
  
  for(list<CDesignator*>::iterator itDesig = lstDesigs.begin();
      itDesig != lstDesigs.end();
      itDesig++) {
    delete *itDesig;
  }
  
  return nID;
}

void BeliefstateClient::endContext(int nContextID, bool bSuccess) {
  CDesignator* desigRequest = new CDesignator();
  desigRequest->setType(ACTION);
  
  desigRequest->setValue(string("_id"), nContextID);
  desigRequest->setValue(string("_success"), (bSuccess ? 1 : 0));
  desigRequest->setValue(string("_source"), m_strSource);
  
  list<CDesignator*> lstDesigs = this->callService(m_sclEndContextService,
						   desigRequest);
  
  for(list<CDesignator*>::iterator itDesig = lstDesigs.begin();
      itDesig != lstDesigs.end();
      itDesig++) {
    delete *itDesig;
  }

  delete desigRequest;
}

list<CDesignator*> BeliefstateClient::alterContext(CDesignator* desigAlter) {
  desigAlter->setValue(string("_source"), m_strSource);
  
  return this->callService(m_sclAlterContextService, desigAlter);
}

void BeliefstateClient::exportFiles(string strFilename) {
  CDesignator* desigRequest = new CDesignator();
  desigRequest->setType(ACTION);
  
  desigRequest->setValue(string("command"), "export-planlog");
  desigRequest->setValue(string("format"), "owl");
  desigRequest->setValue(string("filename"), strFilename + ".owl");
  desigRequest->setValue(string("show-successes"), 1);
  desigRequest->setValue(string("show-fails"), 1);
  desigRequest->setValue(string("max-detail-level"), 99);
  
  this->callService(m_sclAlterContextService, desigRequest);
  
  desigRequest->setValue(string("format"), "dot");
  desigRequest->setValue(string("filename"), strFilename + ".dot");
  
  this->callService(m_sclAlterContextService, desigRequest);
  
  delete desigRequest;
}
