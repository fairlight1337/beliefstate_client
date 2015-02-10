#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <beliefstate_client/BeliefstateClient.h>
#include <beliefstate_client/Context.h>


using namespace beliefstate_client;


int main(int argc, char** argv) {
  ros::init(argc, argv, "bs_client");
  BeliefstateClient* bscl = new BeliefstateClient("bs_client");
  
  bscl->setMetaDataField("experiment", "ex1");
  
  bscl->startNewExperiment();
  bscl->registerOWLNamespace("sim", "http://some-namespace.org/#");
  
  Context* ctxMain = new Context(bscl, "MainTimeline", "&sim;", "MainTimeline", 0);
  ctxMain->annotateParameter("Room", "Kitchen");
  
  Object* objCup = new Object("&sim;", "Cup");
  Object* objTable = new Object("&sim;", "Table");
  Object* objHand = new Object("&sim;", "Hand");
  
  Context* ctxCupOnTable = ctxMain->startContext("ObjectsInContact", "&sim;", "ObjectsInContact", 0);
  ctxCupOnTable->addObject(objCup, "knowrob:objectInContact");
  ctxCupOnTable->addObject(objTable, "knowrob:objectInContact");
  
  Context* ctxCloseHand = ctxMain->startContext("CloseHand", "&sim;", "CloseHand", 1);
  ctxCloseHand->addObject(objHand, "knowrob:affectedObject");
  ctxCloseHand->end(true, 2);
  
  Context* ctxCupInHand = ctxMain->startContext("CupInHand", "&sim;", "CupInHand", 1);
  ctxCupInHand->addObject(objCup, "knowrob:objectInContact");
  ctxCupInHand->addObject(objHand, "knowrob:objectInContact");
  
  Context* ctxLiftHand = ctxMain->startContext("LiftHand", "&sim;", "LiftHand", 3);
  ctxLiftHand->addObject(objHand, "knowrob:affectedObject");
  ctxLiftHand->end(true, 4);
  
  ctxCupOnTable->end(true, 3);
  
  // Cleanup
  ctxCupInHand->end(true, 10);
  ctxMain->end(true, 10);
  
  delete objCup;
  delete objTable;
  
  delete ctxMain;
  delete ctxCupOnTable;
  delete ctxCupInHand;
  delete ctxCloseHand;
  delete ctxLiftHand;
  
  bscl->exportFiles("test");
  
  delete bscl;
  
  return EXIT_SUCCESS;
}
