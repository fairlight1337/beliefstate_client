#include <iostream>
#include <cstdlib>
#include <beliefstate_client/beliefstate_client.h>


int main(int argc, char** argv) {
  BeliefstateClient* bsClient = new BeliefstateClient(argc, argv, "test_source");
  
  int nID = bsClient->startContext("test-context");
  // Do stuff in context.
  bsClient->endContext(nID, true);
  
  delete bsClient;
  
  return EXIT_SUCCESS;
}
