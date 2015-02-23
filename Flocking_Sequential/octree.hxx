#ifndef OCTREE
#define OCTREE

#include <list> 
#include <vector>
#include "vector.hxx"
#include "types.hxx"
#include "agent.hxx"
//class Agent;
class Octree;

typedef std::vector<Octree*> LeafContainer;
class Octree {

public:	
//private:
	/* Ordre spatial à respecter x,y,z croissant successif */
	Octree *child[8];
	Octree *parent;
	Real width; //width of the spatial cell
	Vector position; //x,y,z les plus petits
	int index; // index position in its parent node


	TemporaryContainer agents; // we may use a different data structure than for neighbours
	static Real widthmin;
	static LeafContainer leafs;
	
	Octree(){Octree(0,0);};
	Octree(Real wmin, Real width);
	Octree(Real width, Octree *parent, Vector &position, int index);
	/* Add an agent to its corresponding leaf 
	BEWARE add(reference) but save pointers */
	void add(Agent &a);


	/* Return all the neighbours of an agent without itself */
	void returnNeighboursLeaf(TemporaryContainer &neighbours);
	void add_neighbours( Octree *parent, Vector pos_leaf,TemporaryContainer &neighbours);

	/* TODO return smartly the next agent to compute 
	to avoid re-computing too many distances */

	bool isAllNull();

	void  delete_leaves();
};

#endif