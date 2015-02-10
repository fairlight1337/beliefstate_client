#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <beliefstate_client/BeliefstateClient.h>
#include <beliefstate_client/Context.h>


using namespace beliefstate_client;


int main(int argc, char** argv) {
  ros::init(argc, argv, "bs_client");
  BeliefstateClient* bscl = new BeliefstateClient("bs_client");
  
  bscl->setMetaDataField("experiment", "ex_repeat_offset");
  bscl->registerOWLNamespace("sim", "http://some-namespace.org/#");

  int nOffset = 100;
  
  Context* ctxMain = new Context(bscl, "MainTimeline", "&sim;", "MainTimeline", nOffset);
  ctxMain->annotateParameter("Room", "Kitchen");
  
  for(int nI = 0; nI < 10; nI++) {
    if(nI > 0) {
        bscl->startNewExperiment();
    }
    
    Object* objCup = new Object("&sim;", "Cup");
    Object* objTable = new Object("&sim;", "Table");
    Object* objHand = new Object("&sim;", "Hand");
    
    Context* ctxCupOnTable = ctxMain->startContext("ObjectsInContact", "&sim;", "ObjectsInContact", nOffset);
    ctxCupOnTable->addObject(objCup, "knowrob:objectInContact");
    ctxCupOnTable->addObject(objTable, "knowrob:objectInContact");
    
    Context* ctxCloseHand = ctxMain->startContext("CloseHand", "&sim;", "CloseHand", nOffset + 1);
    ctxCloseHand->addObject(objHand, "knowrob:affectedObject");
    ctxCloseHand->end(true, nOffset + 2);
    
    Context* ctxCupInHand = ctxMain->startContext("CupInHand", "&sim;", "CupInHand", nOffset + 1);
    ctxCupInHand->addObject(objCup, "knowrob:objectInContact");
    ctxCupInHand->addObject(objHand, "knowrob:objectInContact");
    
    Context* ctxLiftHand = ctxMain->startContext("LiftHand", "&sim;", "LiftHand", nOffset + 3);
    ctxLiftHand->addObject(objHand, "knowrob:affectedObject");
    ctxLiftHand->end(true, nOffset + 4);
    
    ctxCupOnTable->end(true, nOffset + 3);
    
    // Cleanup
    ctxCupInHand->end(true, nOffset + 10);
    ctxMain->end(true, nOffset + 10);
    
    delete objCup;
    delete objTable;
    
    delete ctxCupOnTable;
    delete ctxCupInHand;
    delete ctxCloseHand;
    delete ctxLiftHand;
    
    sleep(1);
    bscl->exportFiles("ex_repeat_offset");
    sleep(1);
  }
  
  delete ctxMain;
  
  delete bscl;
  
  return EXIT_SUCCESS;
}
