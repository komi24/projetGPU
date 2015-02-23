/*
*/
#ifndef  WORKSPACE
#define  WORKSPACE

#include "octree.hxx"
#include "parser.hxx"
#include "types.hxx"

//class Octree;

class Workspace
{
//protected:
public:
  Container agents;
  /* Spatial data structure for agent search */
 
  
  unsigned int na;

  Real dt;
  int time;
  Real wCohesion, wAlignment, wSeparation;
  Real rCohesion, rAlignment, rSeparation;
  Real maxU;

  Real tUpload, tDownload, tCohesion, tAlignment, tSeparation;

  Real domainsize;
  
//public:
  Octree oc;
  //Workspace(){};
  Workspace(ArgumentParser &parser);

  Workspace(size_t nAgents,
  Real wc, Real wa, Real ws,
  Real rc, Real ra, Real rs);

  void init();

  /* Update : move agent that needed
  to be moved */
  void update();

void returnNeighboursBuffer(TemporaryContainer &nb, Agent *agent,
  Real rc, TemporaryContainer &bufC,
  Real ra, TemporaryContainer &bufA,
  Real rs, TemporaryContainer &bufS);
  
  void move(int step);
  void move2(int step);
  void simulate(int nsteps);
  void save(int stepid);
};

#endif
