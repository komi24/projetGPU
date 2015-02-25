#ifndef AGENT_HXX
#define AGENT_HXX

#include <deque>
#include <vector>
#include "types.hxx"
#include "vector.cuh"
#include <cuda.h>

typedef enum {
  prey,
  predator,
  active,
  wall
} AgentType;

class Octree;

class Agent;

typedef std::vector<Agent*> TemporaryContainer;


class Agent{
  public :
    /* Current position and next position according to pos_state in octree*/
    Vector position[2]; // TODO set of position should change storage in octree.
    /* Current velocity and next velocity according to pos_state in octree*/
    Vector velocity[2];
    /* Current direction and next direction according to pos_state in octree*/
    Vector direction[2];
    /* Leaf. Leaf 2 gives the new position in octree. according to the set of position */
    Octree* leaf[2];
    /* Indicate if the current position of an agent is from postion 
    or position2 */
    static int curr_state;


    Agent(const Vector &pos, const Vector &vel, const Vector &dir);

    __device__  Vector separation(Agent *agent_list, int sizeNeigh, 
            Real *dist, Real rad, int curr); 
    __device__  Vector cohesion(Agent *agent_list, int sizeNeigh, 
            Real *dist, Real rad, int curr); 
    __device__  Vector alignment(Agent *agent_list, int sizeNeigh, 
            Real *dist, Real rad, int curr); 
    //size_t find_closest(Container &agent_list, size_t index);

/*    bool operator==( const Vector& rhs ) const {
      return true;
    }

    bool operator!=( const Vector& rhs ) const {
      return !operator==( rhs );
    }*/

};
typedef std::deque<Agent> Container; //deque



#endif
