#include <iostream>
#include <cstdlib>
#include <beliefstate_client/beliefstate_client.h>
#include <vector>

using namespace std;


int main(int argc, char** argv) {
  BeliefstateClient* bsClient = new BeliefstateClient(argc, argv, "test_source");
  vector<int> vecIDStack;
  
  vecIDStack.push_back(bsClient->startContext("ANONYMOUS-TOP-LEVEL"));
  vecIDStack.push_back(bsClient->startContext("WITH-DESIGNATORS"));
  vecIDStack.push_back(bsClient->startContext("UIMA-PERCEIVE"));
  
  CDesignator* cdPerceptionRequest = new CDesignator();
  cdPerceptionRequest->setType(OBJECT);
  cdPerceptionRequest->setValue("type", "pancake-mix");
  
  CKeyValuePair* ckvpAt = cdPerceptionRequest->addChild("at");
  ckvpAt->setValue("_designator_type", "LOCATION");
  ckvpAt->setValue("ON", "CUPBOARD");
  ckvpAt->setValue("NAME", "kitchen_island");
  
  CDesignator* cdAction = new CDesignator();
  cdAction->setType(ACTION);
  cdAction->setValue("COMMAND", "ADD-DESIGNATOR");
  cdAction->setValue("TYPE", "ACTION");
  cdAction->setValue("ANNOTATION", "");
  cdAction->setValue("MEMORY-ADDRESS", "abcd1234");
  cdAction->addChild("DESCRIPTION", LIST, cdPerceptionRequest->description());
  
  cdAction->printDesignator();
  
  bsClient->alterContext(cdAction);
  delete cdPerceptionRequest;
  delete cdAction;
  
  bsClient->endContext(vecIDStack.back(), true);
  vecIDStack.pop_back();

  bsClient->endContext(vecIDStack.back(), true);
  vecIDStack.pop_back();

  bsClient->endContext(vecIDStack.back(), true);
  vecIDStack.pop_back();
  
  bsClient->exportFiles("test");
  
  delete bsClient;
  
  return EXIT_SUCCESS;
}
