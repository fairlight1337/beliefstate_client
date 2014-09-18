#include <beliefstate_client/Context.h>


namespace beliefstate_client {
  Context::Context(int nContextID) {
    this->setID(nContextID);
  }
  
  Context::~Context() {
  }
  
  void Context::setID(int nContextID) {
    m_nContextID = nContextID;
  }
  
  int Context::id() {
    return m_nContextID;
  }
}
