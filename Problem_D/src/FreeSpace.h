#ifndef FREESPACE_H
#define FREESPACE_H
#include "parser.h"
#include "SweepLine.h"

class FreeSpace
{
public:

	FreeSpace(vector<Component*>& c, vector<Bound*>& b, Parser& p, vector<Component*>& deletedComp) : 
	comp(c), bound(b), parser(p), _deletedComp(deletedComp)
	{ opt = true; }
	
	~FreeSpace() {}

	// Get
	unordered_set<Component*> getFreeSpace()	{ return free_space; }
	unordered_set<Component*> getNotFreeSpace()	{ return not_free_space; }
	
	//
	void vertical_freeSpace();
	void horizon_freeSpace();
	void insertFreeSpace(int lower_x, int lower_y, int width, int height, bool type);
	void findFreeSpace();
	bool buffer_area_reservation(const double _baredc);

	void getHorizontalBound(Component&, Component&, Component* target);
	void getVerticalBound(Component&, Component&, Component* target);
	void optimizeHorizontalFreeSpace(Node*, Node*, Component* target);
	void optimizeVerticalFreeSpace(Node* , Node*, Component* target);

public:
	Parser& parser;
	vector<Component*>& comp;
	vector<Bound*>& bound;
	vector<Component*>& _deletedComp;
	unordered_set<Component*> free_space;
	unordered_set<Component*> not_free_space;

	bool opt;

};

#endif // !FREESPACE_H