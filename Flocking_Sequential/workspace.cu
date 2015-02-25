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

#include <cuda.h>


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
      
	Agent *agt = new Agent(position,Zeros(),Zeros());
        oc.add(*agt);
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

Agent *Workspace::tempToArray(TemporaryContainer tp){
 std::cerr << " CPU" << std::endl;
  Agent *res = (Agent*) malloc(tp.size()*sizeof(Agent));
  for(int i =0; i<tp.size(); i++){
    res[i]=*tp[i];
    std::cerr << tp[i]->position[0].x << " CPU" << std::endl;
  }

  return res;
}

void Workspace::arrayToTemp(Agent *agts, int s,TemporaryContainer &leaf){
  leaf.clear();
 std::cerr << " GPU" << std::endl;
  for(int i =0; i<s; i++)
  {
    leaf.push_back(&agts[i]);
    std::cerr << agts[i].position[0].x << " GPU" << std::endl;
  }
    
}


__global__ void computeOnGPU(Real r, Agent *agts){
  int *t = (int*) agts;
  for (int i=0; i<sizeof(Agent)/sizeof(int); i++)
      t[i]=0;
  
}

void Workspace::move(int step)//TODO erase step (just for tests)
{
  Vector s,c,a;  
  LeafContainer leafs = Octree::leafs;
  TemporaryContainer nb;
  //std::cout << " leaves "<< Octree::leafs.size() << std::endl;
  for (size_t i=0; i<leafs.size(); i++){
    Octree *it=leafs[i];
    nb.clear();
    (it)->returnNeighboursLeaf(nb);
    TemporaryContainer agentsleaf = (it)->agents;

    //Chargement mémoire sur GPU
    Agent *neighArray=tempToArray(nb);
    Agent *leafArray=tempToArray(leafs[i]->agents);
    Agent *d_neighArray;
    Agent *d_leafArray;
    //TODO penser à supprimer les liste d'agents copiés
    cudaMalloc((void **)&d_neighArray,sizeof(Agent)*nb.size());
    cudaMalloc((void **)&d_leafArray,sizeof(Agent)*leafs[i]->agents.size());
    cudaMemcpy(d_neighArray,neighArray,sizeof(Agent)*nb.size(), cudaMemcpyHostToDevice);
    cudaMemcpy(d_leafArray,leafArray,sizeof(Agent)*leafs[i]->agents.size(), cudaMemcpyHostToDevice);

    //Initialiser la grille
    dim3 dimGrid(1,1,1);
    dim3 dimBlock(1,1,1);

    computeOnGPU<<<dimGrid,dimBlock>>>(0.25,d_leafArray);
    cudaThreadSynchronize();
    cudaMemcpy(d_leafArray,leafArray,sizeof(Agent)*leafs[i]->agents.size(), cudaMemcpyDeviceToHost);
    arrayToTemp(leafArray,leafs[i]->agents.size(),leafs[i]->agents);
      for(size_t j=0; j<agentsleaf.size(); j++){
       Agent *it2=agentsleaf[j];
      /* TemporaryContainer bufA,bufC,bufS;
       returnNeighboursBuffer(nb, it2,
        rCohesion, bufC,
        rAlignment, bufA,
        rSeparation, bufS);
       s = (it2)->separation(bufS, rSeparation);
       c = (it2)->cohesion(bufC, rCohesion);
       a = (it2)->alignment(bufA, rAlignment);
       (it2)->direction[1-Agent::curr_state] = wCohesion*c + wAlignment*a + wSeparation*s;

       (it2)->velocity[1-Agent::curr_state] = (it2)->velocity[Agent::curr_state] + (it2)->direction[1-Agent::curr_state];

       double speed = (it2)->velocity[1-Agent::curr_state].norm();
       if ((speed > maxU)) {
          (it2)->velocity[1-Agent::curr_state] = (it2)->velocity[1-Agent::curr_state] * maxU/speed;
      }

      (it2)->position[1-Agent::curr_state] = (it2)->position[Agent::curr_state] + dt*(it2)->velocity[Agent::curr_state];
*/
      (it2)->position[1-Agent::curr_state].x= fmod((it2)->position[1-Agent::curr_state].x,domainsize);
      (it2)->position[1-Agent::curr_state].y= fmod((it2)->position[1-Agent::curr_state].y,domainsize);
      (it2)->position[1-Agent::curr_state].z= fmod((it2)->position[1-Agent::curr_state].z,domainsize);

    }

    }

    //Agent::curr_state = 1 - Agent::curr_state;
    update();
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
  LeafContainer leafs = Octree::leafs;

   for (size_t i=0; i<leafs.size(); i++){
    Octree *lf=leafs[i];
      for (size_t j = 0; j < lf->agents.size(); j++){
        if ((lf->position > lf->agents[j]->position[Agent::curr_state]) ||
         (lf->agents[j]->position[Agent::curr_state] >= (lf->position + Vector(1,1,1)*lf->width))){      
        lf->agents.erase(std::find(lf->agents.begin(),
          lf->agents.end(),
          lf->agents[j]));
        //lf->agents.remove(&agents[k]);
        lf->delete_leaves();
        oc.add(*lf->agents[j]);
      } else {
         lf->agents[j]->leaf[Agent::curr_state]=lf;
      }   
    }
      }

  /*for(size_t k = 0; k< na; k++){
    if (Agent::curr_state)
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
    }*/
  }


void Workspace::simulate(int nsteps) {
  // store initial position[Agent::curr_state]s
    save(0);

    // perform nsteps time steps of the simulation
    int step = 0;
    while (step++ < nsteps) {
    //std::cout << "coco" << step << std::endl; 
      this->move(step);
      //tst.printOctree(& this->oc);
      // store every 20 steps
      if (step%1 == 0) save(step);
    }
}

void Workspace::save(int stepid) {
  std::ofstream myfile;
  LeafContainer leafs=Octree::leafs;
  myfile.open("boids2.xyz", stepid==0 ? std::ios::out : std::ios::app);

    myfile << std::endl;
    myfile << na << std::endl;
    for (size_t i=0; i<leafs.size(); i++){
    Octree *lf=leafs[i];
      for (size_t j = 0; j < lf->agents.size(); j++)
        myfile << "B " << lf->agents[j]->position[Agent::curr_state];
}
    myfile.close();
}

