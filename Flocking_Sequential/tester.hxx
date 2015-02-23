#ifndef TESTER
#define TESTER

#include "agent.hxx"



class Tester{
public:
	void testConstruction();
	//void testConstruction2();
	void printOctree(Octree *oc);
	void printChild(Octree *oc, int p);
	void printContainer(TemporaryContainer &c);
	void testUpdate();
};

#endif