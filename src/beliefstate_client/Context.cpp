#include <beliefstate_client/Context.h>


namespace beliefstate_client {
  Context::Context(BeliefstateClient* bsclClient, std::string strContextName, int nTimeStamp) {
    m_bsclClient = bsclClient;
    
    this->setID(bsclClient->startContext(strContextName, nTimeStamp));
  }
  
  Context::Context(BeliefstateClient* bsclClient, std::string strContextName, std::string strClassNamespace, std::string strClass, int nTimeStamp) {
    m_bsclClient = bsclClient;
    
    this->setID(m_bsclClient->startContext(strContextName, strClassNamespace, strClass, nTimeStamp));
  }
  
  Context::~Context() {
  }
  
  void Context::end(bool bSuccess, int nTimeStamp) {
    m_bsclClient->endContext(this->id(), bSuccess, nTimeStamp);
  }
  
  void Context::setID(int nContextID) {
    m_nContextID = nContextID;
  }
  
  int Context::id() {
    return m_nContextID;
  }
}
