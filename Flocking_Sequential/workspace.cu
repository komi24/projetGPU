#include <cmath>
#include <iostream>
#include <fstream>
#include <ctime>
#include <omp.h>

#include "workspace.hxx"
#include "agent.cuh"
#include "vector.cuh"
#include "tester.hxx"
//#include "octree.hxx"

#include <cuda.h>

#define BUFF_SIZE 20

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
    //std::cerr << tp[i]->position[Agent::curr_state].x << " CPU" << std::endl;
   
  }

  return res;
}

void Workspace::arrayToTemp(Agent *agts, int s,TemporaryContainer &leaf){
  leaf.clear();
 std::cerr << " GPU" << std::endl;
  for(int i =0; i<s; i++)
  {
    leaf.push_back(&agts[i]);
    //std::cerr << agts[i].position[Agent::curr_state].x << " GPU" << std::endl;
    //std::cerr << agts[i].position[1- Agent::curr_state].x << " 1- curr GPU" << std::endl;
   
  }
    
}



__device__  Vector separation(Agent &a, Agent *agent_list, int sizeNeigh, Real *dist, Real rad, int curr) {

    Vector force = Vector();
    int count =0;
    for(size_t i = 0; i < sizeNeigh; i++) {
        //double dist = (a.position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
        if ((dist[i] < rad) && (0<dist[i])) {
            // TODO the comparison is no longer needed //
            force -= (a.position[curr] - agent_list[i].position[curr]).normalized();
            ++count;
        }
    }
    return ( count >0 ? force/count : force);

}


__device__ Vector cohesion(Agent &a,Agent *agent_list, int sizeNeigh, Real *dist, Real rad, int curr) {

    Vector force;
    int count = 0;
    for(size_t i = 0; i < sizeNeigh; i++) {
        //double dist = (a.position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
        if ((dist[i] < rad) && (0<dist[i])) {
            // TODO the comparison is no longer needed //
            force += agent_list[i].position[curr];
            ++count;
        }
    }
    return ( count >0 ? force/count : force);
}

__device__ Vector alignment(Agent &a,Agent *agent_list, int sizeNeigh, Real *dist, Real rad, int curr) {
    Vector force;
    int count = 0;
    for(size_t i = 0; i < sizeNeigh; i++) {
        if ((dist[i] < rad) && (0<dist[i])) {
            // TODO the comparison is no longer needed //
            force += agent_list[i].velocity[curr];
            ++count;
        }
    }
    return ( count >0 ? force/count : force);
}

__global__ void computeOnGPU(int sizeNb, int sizeLf, 
        Agent *agts, Agent *neigh, 
        Real rs, Real rc, Real ra, 
        Real wSeparation, Real wCohesion, Real wAlignment, 
        int curr, Real maxU,Real dt){
    int tileWidth = sizeNb/sizeLf;
    __shared__ Real ds_neighInst[BUFF_SIZE*sizeof(Agent)/sizeof(Real)];//TODO mettre à zero les champs
    __shared__ Agent *ds_neigh;
    ds_neigh = (Agent *) ds_neighInst;
    //ds_neigh = (Agent*) malloc(sizeof(Agent)*(tileWidth)); // TODO Faire gaffe un seul thread
    __shared__ Real ds_dist[BUFF_SIZE];
    //ds_dist = (Real *) malloc(sizeof(Real)*(tileWidth));
    Vector s, c, a;

    for (int j=0; j<sizeLf; j++){
        //Chargement mémoire
        for (int i= 0; i<tileWidth; i++){
            ds_neigh[i]=neigh[(blockIdx.x+j)*tileWidth+i];
        }
        __syncthreads();
        //Calcul des distances
        for (int i= 0; i<tileWidth; i++){
            ds_dist[i]=(agts[blockIdx.x].position[curr]-ds_neigh[(blockIdx.x+j)*tileWidth+i].position[curr]).norm();//TODO passer norm en __global__
        }
        __syncthreads();
        //Calcul des forces 
        //s =
        s =  separation(agts[blockIdx.x],ds_neigh,tileWidth, ds_dist, rs, curr);
        c = cohesion(agts[blockIdx.x],ds_neigh,tileWidth, ds_dist, rc, curr);
        a = alignment(agts[blockIdx.x],ds_neigh,tileWidth, ds_dist, ra, curr);

        agts[blockIdx.x].direction[1-curr] = c*wCohesion + a*wAlignment + s*wSeparation;

        agts[blockIdx.x].velocity[1-curr] = agts[blockIdx.x].velocity[curr] 
            + agts[blockIdx.x].direction[1-curr];
        float speed =agts[blockIdx.x].velocity[1-curr].norm();
        if ((speed > maxU)) {
            agts[blockIdx.x].velocity[1-curr] = agts[blockIdx.x].velocity[1-curr] * maxU/speed;
        }
        agts[blockIdx.x].position[1-curr] = agts[blockIdx.x].position[curr] + agts[blockIdx.x].velocity[curr]*dt;

        __syncthreads();
    }
  
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

    //TODO penser à supprimer les liste d'agents copiées
    cudaMalloc((void **)&d_neighArray,sizeof(Agent)*nb.size());
    cudaMalloc((void **)&d_leafArray,sizeof(Agent)*leafs[i]->agents.size());
    cudaMemcpy(d_neighArray,neighArray,sizeof(Agent)*nb.size(), cudaMemcpyHostToDevice);
    cudaMemcpy(d_leafArray,leafArray,sizeof(Agent)*leafs[i]->agents.size(), cudaMemcpyHostToDevice);
    
    //Initialiser la grille
    dim3 dimGrid(leafs[i]->agents.size(),1,1);
    dim3 dimBlock(1,1,1);

    computeOnGPU<<<dimGrid,dimBlock>>>(nb.size(), leafs[i]->agents.size(), 
        d_leafArray, d_neighArray, 
         rSeparation,  rCohesion,  rAlignment, 
         wSeparation,  wCohesion,  wAlignment, 
         Agent::curr_state,maxU,dt);
    cudaError err = cudaGetLastError();
    if(cudaSuccess != err )
        std::cerr << cudaGetErrorString(err) << std::endl;
    cudaThreadSynchronize();
    cudaMemcpy(leafArray,d_leafArray,sizeof(Agent)*leafs[i]->agents.size(), cudaMemcpyDeviceToHost);

  
    arrayToTemp(leafArray,leafs[i]->agents.size(),leafs[i]->agents);

    cudaFree(d_neighArray);
    //cudaFree(neighArray);
    cudaFree(d_leafArray);
    //cudaFree(leafArray);

    cudaThreadSynchronize();

      for(size_t j=0; j<agentsleaf.size(); j++){

       Agent *it2=leafs[i]->agents[j];

      (it2)->position[1-Agent::curr_state].x= fmod((it2)->position[1-Agent::curr_state].x,domainsize);
      (it2)->position[1-Agent::curr_state].y= fmod((it2)->position[1-Agent::curr_state].y,domainsize);
      (it2)->position[1-Agent::curr_state].z= fmod((it2)->position[1-Agent::curr_state].z,domainsize);

    }

    }

    Agent::curr_state = 1 - Agent::curr_state;
    std::cerr << "ok1" << std::endl;
    update();
    std::cerr << "ok2" << std::endl;
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
        oc.add(*lf->agents[j]);
            std::cerr << "test3" << std::endl;
        lf->delete_leaves();
            std::cerr << "test4" << std::endl;
      } else {
            std::cerr << "test5" << std::endl;
         lf->agents[j]->leaf[Agent::curr_state]=lf;
            std::cerr << "test5" << std::endl;
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
    std::cerr << "ok3" << std::endl;
      //tst.printOctree(& this->oc);
      // store every 20 steps
      if (step%1 == 0) save(step);
    }
}

void Workspace::save(int stepid) {
  std::ofstream myfile;
  LeafContainer leafs=Octree::leafs;
    std::cerr << "ok4" << std::endl;
  myfile.open("boids.xyz", stepid==0 ? std::ios::out : std::ios::app);
    std::cerr << "ok4" << std::endl;

    myfile << std::endl;
    myfile << na << std::endl;
    for (size_t i=0; i<leafs.size(); i++){
    Octree *lf=leafs[i];
      for (size_t j = 0; j < lf->agents.size(); j++)
        myfile << "B " << lf->agents[j]->position[Agent::curr_state];
}
    myfile.close();
}

