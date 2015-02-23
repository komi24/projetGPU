#include <cmath>
#include <iostream>
#include <fstream>
#include <ctime>
#include <omp.h>

#include "workspace.hxx"
#include "agent.hxx"
#include "vector.hxx"
#include "tester.hxx"
//#include "octree.hxx"


Workspace::Workspace(ArgumentParser &parser)
{

  na = parser("agents").asInt();

  wCohesion = parser("wc").asDouble();
  wAlignment = parser("wa").asDouble();
  wSeparation = parser("ws").asDouble();

  rCohesion = parser("rc").asDouble();
  rAlignment = parser("ra").asDouble();
  rSeparation = parser("rs").asDouble();
  dt= 0.05;
  maxU = 2.0;
  time = 0.,//;

  this->init();}

Workspace::Workspace(size_t nAgents,
             Real wc, Real wa, Real ws,
             Real rc, Real ra, Real rs) :
             na(nAgents), dt(.05), time(0),
             wCohesion(wc), wAlignment(wa), wSeparation(ws),
             rCohesion(rc), rAlignment(ra), rSeparation(rs),
             maxU(2.)
{ this->init();}

void  Workspace::init(){
    domainsize = 1.0;

    // Random generator seed
    srand48(std::time(0));

    //Initializing Octree head
    Real maxR;
    maxR = (rCohesion > rSeparation) ? rCohesion : rSeparation;
    maxR = (maxR > rAlignment) ? maxR : rAlignment;
    oc = *(new Octree(2*maxR,domainsize));

    // Initialize agents
    // This loop may be quite expensive due to random number generation
    //agents.reserve(na);
    //#pragma omp parallel
    //{
      //#pragma omp for
      for(size_t j = 0; j < na; j++){
      // Create random position
        Vector position(drand48(), drand48(), drand48());

      // Create random velocity
      //agents.push_back(Agent(position, Zeros(), Zeros()));
        agents.push_back(Agent(position, Zeros(), Zeros()));
        oc.add(agents[j]);
      }
    //}

    /*for(size_t j = 0; j < na; j++){
      // Create random position
      Vector position(drand48(), drand48(), drand48());

      // Create random velocity
      //agents.push_back(Agent(position, Zeros(), Zeros()));
      //agents.assign(j,Agent(position, Zeros(), Zeros()));
      oc.add(agents[j]);
    }*/

    /* TODO build the octree */

}

void Workspace::move(int step)//TODO erase step (just for tests)
{
  Vector s,c,a;  
  LeafContainer leafs = Octree::leafs;
  TemporaryContainer nb;
  //std::cout << " leaves "<< Octree::leafs.size() << std::endl;
  #pragma omp parallel private(s,c,a,nb) 
  {
    #pragma omp for
    for (size_t i=0; i<leafs.size(); i++){
    //for (LeafContainer::iterator it = leafs.begin(); it != leafs.end(); it++){
      Octree *it=leafs[i];
      nb.clear();
      //std::cout << "a" << omp_get_thread_num() << std::endl;
      (it)->returnNeighboursLeaf(nb);
      //std::cout << " voisins "<< nb.size() << std::endl;
      TemporaryContainer agentsleaf = (it)->agents;
      #pragma omp parallel private(s,c,a)
      {
      #pragma omp for 
      for(size_t j=0; j<agentsleaf.size(); j++){
      //for (TemporaryContainer::iterator it2 = agentsleaf.begin(); it2 != agentsleaf.end(); it2++){
      //std::cout << "b" << omp_get_thread_num() << std::endl;
       Agent *it2=agentsleaf[j];
       TemporaryContainer bufA,bufC,bufS;
       returnNeighboursBuffer(nb, it2,
        rCohesion, bufC,
        rAlignment, bufA,
        rSeparation, bufS);
       s = (it2)->separation(bufS, rSeparation);
        //}
      //#pragma omp section
        //{
       c = (it2)->cohesion(bufC, rCohesion);
        //}
      //#pragma omp section
        //{
       a = (it2)->alignment(bufA, rAlignment);
       (it2)->direction[1-Agent::curr_state] = wCohesion*c + wAlignment*a + wSeparation*s;

       (it2)->velocity[1-Agent::curr_state] = (it2)->velocity[Agent::curr_state] + (it2)->direction[1-Agent::curr_state];

       double speed = (it2)->velocity[1-Agent::curr_state].norm();
       if ((speed > maxU)) {
          (it2)->velocity[1-Agent::curr_state] = (it2)->velocity[1-Agent::curr_state] * maxU/speed;
        //std::cout << " maxU/speed " << maxU/speed <<std::endl;
      }

      (it2)->position[1-Agent::curr_state] = (it2)->position[Agent::curr_state] + dt*(it2)->velocity[Agent::curr_state];

      //std::cout << " direction " << agents[k].direction[Agent::curr_state] <<std::endl;
      //std::cout << " velocity " << agents[k].velocity[Agent::curr_state] <<std::endl;
      //std::cout << " speed " << speed <<std::endl;
      //std::cout << " position " << agents[k].position[Agent::curr_state] <<std::endl;

      (it2)->position[1-Agent::curr_state].x= fmod((it2)->position[1-Agent::curr_state].x,domainsize);
      (it2)->position[1-Agent::curr_state].y= fmod((it2)->position[1-Agent::curr_state].y,domainsize);
      (it2)->position[1-Agent::curr_state].z= fmod((it2)->position[1-Agent::curr_state].z,domainsize);

    }
    }


  }
}


    // Integration in time using euler method
    //TODO Remark for report : parallelism gain thx to curr_state
Agent::curr_state = 1 - Agent::curr_state;
    /*for(size_t k = 0; k< na; k++){
      //std::cout << "ok2 " << k << " na " << na << std::endl; 
      agents[k].velocity[Agent::curr_state] = agents[k].velocity[1-Agent::curr_state] + agents[k].direction[Agent::curr_state];

      double speed = agents[k].velocity[Agent::curr_state].norm();
      if ((speed > maxU)) {
        agents[k].velocity[Agent::curr_state] = agents[k].velocity[Agent::curr_state] * maxU/speed;
        //std::cout << " maxU/speed " << maxU/speed <<std::endl;
      }

      agents[k].position[Agent::curr_state] = agents[k].position[1-Agent::curr_state] + dt*agents[k].velocity[1-Agent::curr_state];

      //std::cout << " direction " << agents[k].direction[Agent::curr_state] <<std::endl;
      //std::cout << " velocity " << agents[k].velocity[Agent::curr_state] <<std::endl;
      //std::cout << " speed " << speed <<std::endl;
      //std::cout << " position " << agents[k].position[Agent::curr_state] <<std::endl;

      agents[k].position[Agent::curr_state].x= fmod(agents[k].position[Agent::curr_state].x,domainsize);
      agents[k].position[Agent::curr_state].y= fmod(agents[k].position[Agent::curr_state].y,domainsize);
      agents[k].position[Agent::curr_state].z= fmod(agents[k].position[Agent::curr_state].z,domainsize);

      //agents[k].velocity[Agent::curr_state].x= fmod(agents[k].velocity[Agent::curr_state].x,10000);
      //agents[k].velocity[Agent::curr_state].y= fmod(agents[k].velocity[Agent::curr_state].y,10000);
      //agents[k].velocity[Agent::curr_state].z= fmod(agents[k].velocity[Agent::curr_state].z,10000);

    }*/

    //std::cout << "caca " << step << std::endl; 
      update();
    //std::cout << "caca " << step  << std::endl; 
    //std::cout << "state " << Agent::curr_state  << std::endl; 
}

void Workspace::returnNeighboursBuffer(TemporaryContainer &nb, Agent *agent,
  Real rc, TemporaryContainer &bufC,
  Real ra, TemporaryContainer &bufA,
  Real rs, TemporaryContainer &bufS
  ){
  for(int i=0; i<nb.size(); i++){
    Real dist =  (agent->position[Agent::curr_state] - nb[i]->position[Agent::curr_state]).norm();
    if(dist <= rc)
      bufC.push_back(agent);
    if(dist <= ra)
      bufA.push_back(agent);
    if(dist <= rs)
      bufS.push_back(agent);
  }  
}



void Workspace::update(){
  //#pragma omp parallel for
  for(size_t k = 0; k< na; k++){
    Octree *lf = agents[k].leaf[1-Agent::curr_state];
    //Retirer de la liste si nécessaire et rajouter au bon endroit
    if((lf->position > agents[k].position[Agent::curr_state]) 
      || (agents[k].position[Agent::curr_state] >= (lf->position + Vector(1,1,1)*lf->width))) {
        lf->agents.erase(std::find(lf->agents.begin(),
          lf->agents.end(),
          &agents[k]));
        //lf->agents.remove(&agents[k]);
        lf->delete_leaves();
        oc.add(agents[k]);
      } else {
        agents[k].leaf[Agent::curr_state]=lf;
      }
    }
  }


void Workspace::simulate(int nsteps) {
  // store initial position[Agent::curr_state]s
    save(0);
    Tester tst;

    // perform nsteps time steps of the simulation
    int step = 0;
    while (step++ < nsteps) {
    //std::cout << "coco" << step << std::endl; 
      this->move(step);
      //tst.printOctree(& this->oc);
      // store every 20 steps
      if (step%5 == 0) save(step);
    }
}

void Workspace::save(int stepid) {
  std::ofstream myfile;

  myfile.open("boids2.xyz", stepid==0 ? std::ios::out : std::ios::app);

    myfile << std::endl;
    myfile << na << std::endl;
    for (size_t p=0; p<na; p++)
        myfile << "B " << agents[p].position[Agent::curr_state];

    myfile.close();
  }
