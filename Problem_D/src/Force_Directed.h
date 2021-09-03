#ifndef FORCE_DIRECTED_H
#define FORCE_DIRECTED_H
#include "SweepLine.h"
#include "TCG.h"
#include <math.h>

#define INF ((1 << 30) - 4)
#define NINF (-INF)

extern vector<Bound*> bound;
extern map<string, vector<Component*>> group;
extern vector<Component*> comp;

class Force_Directed
{
public:
	Force_Directed(int minSpace): _minSpace(minSpace) {}
	~Force_Directed() {}

	// Get
	vector<Component*> getComp() 			{ return comp; }
	vector<vector<Component*>> getGroup() 	{ return _grps; }
	// Set
	void setComp(vector<Component*> c)			{ comp = c; }
	void setGroup(vector<vector<Component*>> g)	{ _grps = g; };

	// Overlapped group
	void addToGroup(vector<vector<Component*>>& _grps, Component* root, Component* added); // add to overlapped group
	void genOverlapGroup();
	void genOverlapGroup_ver2();

	void force_calculation(Component* rc, Component* nc);
	void do_forceDirected_ver2();
	void do_forceDirected();

	// Design Boundary detection
	void limitedInBoundary(Component* c, bool force_dir);
	void m_2_m_force(Component *c, double& _origin, double _new);
	void m_2_b_force(double& _origin, double _new);
	void dw_handler(Component* c);
	bool isInsideBoundary(Component* c);

	// For corner-stitching
	void generate_overlap_group();
	void move_macro_inside_boundary();
	void macro_boundary_checker(Component* c, bool& isAllInBoundary);

private:

	vector<vector<Component*>> _grps; // overlapped group

	// the width of entire design
	// to check if macro is out of design
	vector<pair<int, int>> _dw;
	int _minSpace;
};

#endif