
// Agent (particle model)

//#include "types.hxx"
#include "workspace.hxx"
#include "parser.hxx"
#include "tester.hxx"
//#include "agent.hxx"

#include <ctime>

// Main class for running the parallel flocking sim
int main(int argc, char **argv) {
  // Create parser
  //Tester t;
  //t.testConstruction();
  //t.testUpdate();
  
  ArgumentParser parser;
  Tester tst;
  time_t beg,end;
  beg = time(NULL);

  // Add options to parser
  parser.addOption("agents", 5000);//640 originally
  parser.addOption("steps", 500);//500
  parser.addOption("wc", 12);
  parser.addOption("wa", 15);
  parser.addOption("ws", 35);

  parser.addOption("rc", 0.11);
  parser.addOption("ra", 0.15);
  parser.addOption("rs", 0.01);

  // Parse command line arguments
  parser.setOptions(argc, argv);

  // Create workspace
  Workspace workspace(parser);


  // Launch simulation
  int nSteps = parser("steps").asInt();
  workspace.simulate(nSteps);

  end = time(NULL);
  std::cout << (double)difftime(end,beg) << std::endl;

  return 0;
}
