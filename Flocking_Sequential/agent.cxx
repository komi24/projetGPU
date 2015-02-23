#include "agent.hxx"
#include "octree.hxx"


int Agent::curr_state =0;


Agent::Agent(const Vector &pos, const Vector &vel, const Vector &dir){
//TODO Use of position/velocity/direction lists ? + Parallelisr les opérations
//sur direction/velocity/position
  position[this->curr_state] = pos;
  velocity[this->curr_state] = vel;
  direction[this->curr_state] = dir;
}



Vector Agent::separation(TemporaryContainer &agent_list, double rad) {


Vector force = Zeros();
//int count =0;
    for(size_t i = 0; i < agent_list.size(); i++) {
    //double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    //if (dist < rad) {
      // TODO the comparison is no longer needed //
      force -= (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).normalized();
      //++count;
    }
    //}
  return ( agent_list.size() >0 ? force/agent_list.size() : force);
}

/*Vector Agent::separation(TemporaryContainer &agent_list, double rad) {

  Real forcex = 0;
  Real forcey = 0;
  Real forcez = 0;
  int count = 0;
  #pragma omp parallel 
  {
  #pragma omp for reduction( - : forcex)
    for(size_t i = 0; i < agent_list.size(); i++) {
    double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    if ( dist < rad) {
      // TODO the comparison is no longer needed //
      forcex -= (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).normalized().x;
      ++count;
    }
    }
  #pragma omp for reduction( - : forcey)
    for(size_t i = 0; i < agent_list.size(); i++) {
    double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    if ( dist < rad) {
      // TODO the comparison is no longer needed //
      forcey -= (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).normalized().y;
      //++count;
    }
    }
  #pragma omp for reduction( - : forcez)
    for(size_t i = 0; i < agent_list.size(); i++) {
    double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    if ( dist < rad) {
      // TODO the comparison is no longer needed //
      forcez -= (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).normalized().z;
      //++count;
    }
    }
  }
  Vector force(forcex,forcey,forcez);
  return ( count >0 ? force/count : force);
}*/


Vector Agent::cohesion(TemporaryContainer &agent_list,  double rad) {

Vector force = Zeros();
  //int count = 0;
    for(size_t i = 0; i < agent_list.size(); i++) {
    //double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    //if (dist < rad) {
      // TODO the comparison is no longer needed //
      force += agent_list[i]->position[this->curr_state];
      //++count;
    }
    //}
  return ( agent_list.size() >0 ? force/agent_list.size() : force);
}
/*Vector Agent::cohesion(TemporaryContainer &agent_list, double rad) {

  Real forcex = 0;
  Real forcey = 0;
  Real forcez = 0;
  int count = 0;
  #pragma omp parallel 
  {
  #pragma omp for reduction( + : forcex)
    for(size_t i = 0; i < agent_list.size(); i++) {
    double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    if ( dist < rad) {
      // TODO the comparison is no longer needed //
      forcex += agent_list[i]->position[this->curr_state].x;
      ++count;
    }
    }
  #pragma omp for reduction( + : forcey)
    for(size_t i = 0; i < agent_list.size(); i++) {
    double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    if ( dist < rad) {
      // TODO the comparison is no longer needed //
      forcey += agent_list[i]->position[this->curr_state].y;
      //++count;
    }
    }
  #pragma omp for reduction( + : forcez)
    for(size_t i = 0; i < agent_list.size(); i++) {
    double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    if ( dist < rad) {
      // TODO the comparison is no longer needed //
      forcez += agent_list[i]->position[this->curr_state].z;
      //++count;
    }
    }
  }
  Vector force(forcex,forcey,forcez);
  return ( count >0 ? force/count : force);
}*/


Vector Agent::alignment(TemporaryContainer&agent_list, double rad) {
Vector force = Zeros();
  //int count = 0;
    for(size_t i = 0; i < agent_list.size(); i++) {
    //double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    //if ( dist < rad) {
      // TODO the comparison is no longer needed //
      force += agent_list[i]->velocity[this->curr_state];
      //++count;
    }
    //}
  return ( agent_list.size() >0 ? force/agent_list.size() : force);
}

/*Vector Agent::alignment(TemporaryContainer&agent_list, double rad) {

  Real forcex = 0;
  Real forcey = 0;
  Real forcez = 0;
  int count = 0;
  #pragma omp parallel 
  {
  #pragma omp for reduction( + : forcex)
    for(size_t i = 0; i < agent_list.size(); i++) {
    double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    if ( dist < rad) {
      // TODO the comparison is no longer needed //
      forcex += agent_list[i]->velocity[this->curr_state].x;
      ++count;
    }
    }
  #pragma omp for reduction( + : forcey)
    for(size_t i = 0; i < agent_list.size(); i++) {
    double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    if ( dist < rad) {
      // TODO the comparison is no longer needed //
      forcey += agent_list[i]->velocity[this->curr_state].y;
      //++count;
    }
    }
  #pragma omp for reduction( + : forcez)
    for(size_t i = 0; i < agent_list.size(); i++) {
    double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    if ( dist < rad) {
      // TODO the comparison is no longer needed //
      forcez += agent_list[i]->velocity[this->curr_state].z;
      //++count;
    }
    }
  }
  Vector force(forcex,forcey,forcez);
  return ( count >0 ? force/count : force);
}*/

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









