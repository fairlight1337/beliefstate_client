#include <beliefstate_client/Object.h>


namespace beliefstate_client {
  Object::Object(std::string strNamespace, std::string strClass) : CDesignator(OBJECT) {
    m_strNamespace = strNamespace;
    m_strClass = strClass;
  }
  
  Object::~Object() {
  }
  
  std::string Object::className() {
    return m_strClass;
  }
  
  std::string Object::classNamespace() {
    return m_strNamespace;
  }
}
