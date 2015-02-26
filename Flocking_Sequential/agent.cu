#include "agent.cuh"
#include "octree.hxx"


int Agent::curr_state =0;


__device__ Agent::Agent(const Vector &pos, const Vector &vel, const Vector &dir, const int curr){
//TODO Use of position/velocity/direction lists ? + Parallelisr les opérations
//sur direction/velocity/position
  position[curr] = pos;
  velocity[curr] = vel;
  direction[curr] = dir;
}

Agent::Agent(const Vector &pos, const Vector &vel, const Vector &dir){
//TODO Use of position/velocity/direction lists ? + Parallelisr les opérations
//sur direction/velocity/position
  position[Agent::curr_state] = pos;
  velocity[Agent::curr_state] = vel;
  direction[Agent::curr_state] = dir;
}

__host__ __device__ Agent::Agent(){
}



__device__  Vector  Agent::separation(Agent *agent_list, int sizeNeigh, Real *dist, Real rad, int curr) {

   Vector force;
    int count =0;
    for(size_t i = 0; i < sizeNeigh; i++) {
        //double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
        if ((dist[i] < rad) && (0<dist[i])) {
            // TODO the comparison is no longer needed //
            force -= (this->position[curr] - agent_list[i].position[curr]).normalized();
            ++count;
        }
    }
    return ( count >0 ? force/count : force);
//return force;
}


__device__ Vector Agent::cohesion(Agent *agent_list, int sizeNeigh, Real *dist, Real rad, int curr) {

    Vector force;
    int count = 0;
    for(size_t i = 0; i < sizeNeigh; i++) {
        //double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
        if ((dist[i] < rad) && (0<dist[i])) {
            // TODO the comparison is no longer needed //
            force += agent_list[i].position[curr];
            ++count;
        }
    }
    return ( count >0 ? force/count : force);
}

__device__ Vector Agent::alignment(Agent *agent_list, int sizeNeigh, Real *dist, Real rad, int curr) {
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


/*size_t Agent::find_closest(Container &agent_list, size_t index) {
  size_t closest_agent = index;
  double min_dist = 1000;

  double dist;

  for(size_t i = 0; i < agent_list.size(); i++) {
    if (i != index) {
      dist= (this->position[this->curr_state] - agent_list[i].position[this->curr_state]).norm();

      if(dist < min_dist) {
        min_dist = dist;
        closest_agent = i;
      }
    }
  }
  return closest_agent;
}*/

/* Return all the neighbours of an agent without itself */









