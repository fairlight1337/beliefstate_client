#include <beliefstate_client/Context.h>


namespace beliefstate_client {
  Context::Context(BeliefstateClient* bsclClient, int nContextID) {
    m_bsclClient = bsclClient;
    
    this->setID(nContextID);
  }
  
  Context::Context(BeliefstateClient* bsclClient, std::string strContextName, int nTimeStamp) {
    m_bsclClient = bsclClient;
    
    this->setID(bsclClient->startContext(strContextName, -1, nTimeStamp));
  }
  
  Context::Context(BeliefstateClient* bsclClient, std::string strContextName, std::string strClassNamespace, std::string strClass, int nTimeStamp) {
    m_bsclClient = bsclClient;
    
    this->setID(m_bsclClient->startContext(strContextName, -1, strClassNamespace, strClass, nTimeStamp));
  }
  
  Context::~Context() {
  }
  
  void Context::end(bool bSuccess, int nTimeStamp) {
    m_bsclClient->endContext(this->id(), bSuccess, nTimeStamp, true);
  }
  
  void Context::setID(int nContextID) {
    m_nContextID = nContextID;
  }
  
  int Context::id() {
    return m_nContextID;
  }
  
  Context* Context::startContext(std::string strContextName, std::string strClassNamespace, std::string strClass, int nTimeStamp) {
    return new Context(m_bsclClient, m_bsclClient->startContext(strContextName, this->id(), strClassNamespace, strClass, nTimeStamp));
  }
  
  void Context::annotateParameter(std::string strKey, std::string strValue) {
    m_bsclClient->annotateParameter(strKey, strValue, this->id());
  }
  
  void Context::annotateParameter(std::string strKey, float fValue) {
    m_bsclClient->annotateParameter(strKey, fValue, this->id());
  }
  
  void Context::addObject(Object* objAdd, std::string strProperty) {
    m_bsclClient->addObject(objAdd, strProperty, this->id());
  }
  
  void Context::addDesignator(CDesignator* cdAdd, std::string strAnnotation) {
    m_bsclClient->addDesignator(cdAdd, strAnnotation, this->id());
  }
  
  void Context::discreteEvent(std::string strEventName, std::string strClassNamespace, std::string strClass, bool bSuccess, int nTimeStamp) {
    m_bsclClient->discreteEvent(strEventName, this->id(), strClassNamespace, strClass, bSuccess, nTimeStamp, true);
  }
}
