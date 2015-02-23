#include "tester.hxx"

#include <iostream>
#include <ctime>
#include "octree.hxx"
#include "workspace.hxx"

void Tester::testConstruction(){
	Octree oc = Octree(0.0625,1);
	
	Vector position(0.59, 0.53, 0.53);
	Agent a(position, Zeros(), Zeros());
    oc.add(a);


    Vector position1(0.53, 0.30, 0.53);    
	Agent a1(position1, Zeros(), Zeros());
	oc.add(a1);


	Vector position2(0.59, 0.59, 0.59);    
	Agent a2(position2, Zeros(), Zeros());
	oc.add(a2);

	Vector position3(0.53, 0.53, 0.53);
	Agent a3(position3, Zeros(), Zeros());
    oc.add(a3);

    printOctree(&oc);
    TemporaryContainer x,y,z;
   // a3.returnNeighbours(0.0625,x,0.0625,y,0.0625,z);
    this->printContainer(z);
}

void Tester::testUpdate(){
	Workspace wkspce(3,1,1,1,0.01,0.05,0.02);
	srand48(std::time(0)+100);
	printOctree(&(wkspce.oc));
	Agent::curr_state = 1- Agent::curr_state;
	for (uint k=0; k<wkspce.agents.size(); k++){
		std::cout << " Ancienne position " << wkspce.agents[k].position[1-Agent::curr_state] <<std::endl;
		wkspce.agents[k].position[Agent::curr_state] = Vector(drand48(),drand48(),drand48());
		std::cout << " Nouvelle position " << wkspce.agents[k].position[Agent::curr_state] <<std::endl <<std::endl;
	}
	wkspce.update();
	printOctree(&(wkspce.oc));
}

void Tester::printContainer(TemporaryContainer &c){
	std::cout << " liste des agents"<< std::endl;
	for (TemporaryContainer::iterator it = c.begin(); it != c.end(); it++){
		std::cout << "	Agent " << " position " << (*it)->position[Agent::curr_state] << std::endl;
	}	
}

/*void Tester::testConstruction2(){

	Octree oc = Octree(0.5,1);
	
	Vector position(0, 0, 0); 
	Agent *a1 =  new Agent(position, Zeros(), Zeros());
    oc.add(*a1);

    Vector position(0, 0.5, 0);    
    oc.add(Agent(position, Zeros(), Zeros()));

	Vector position(0, 0, 0.5);    
    oc.add(Agent(position, Zeros(), Zeros()));

	Vector position(0, 0.5, 0.5);    
    oc.add(Agent(position, Zeros(), Zeros()));

    printOctree(&oc);

  
}*/

void Tester::printOctree(Octree *oc){
	std::cout << " Tête " << std::endl ;
	for(int i=0; i<8; i++){
		if(oc->child[i]!=NULL){
				//std::cout << " feuille " << i << std::endl;
				printChild(oc->child[i],0);
			}else{
				//std::cout << " vide " << std::endl;
			}
//		std::cout << "	Fils " << i <<std::endl;
	}
}

void Tester::printChild(Octree *oc, int p){
	if(oc->agents.size()==0){
		for(int i=0; i<8; i++){
			if(oc->child[i]!=NULL){
				//for (int n=0; n<p; n++) std::cout << "	";
				//std::cout << " Suite " << std::endl;
				printChild(oc->child[i], p++);
			}else{
				//std::cout << " vide " << std::endl;
			}
//		std::cout << "	Fils " << i <<std::endl;
		}
	}else{
		std::cout << "Feuille Position " << oc->position << " Width " << oc->width << std::endl;
		for(TemporaryContainer::iterator it = oc->agents.begin(); it != oc->agents.end(); it++){
			//printChild(oc->child[i],i);
			//for (int n=0; n<p+1; n++) std::cout << "	";
			std::cout << "	Agent " << " position " << (*it)->position[Agent::curr_state] << std::endl;
			std::cout << "	Agent " << " velocity " << (*it)->velocity[Agent::curr_state] << std::endl;
		}
	}

}