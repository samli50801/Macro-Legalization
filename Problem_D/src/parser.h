#ifndef PARSER_H
#define PARSER_H
#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>
#include <vector>
#include <fstream>
#include <map>
#include <vector>
#include <queue>
#include <set>
#include <list>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <climits>
#include <float.h>

using namespace std;

//#define DEBUG_MODE

#ifdef DEBUG_MODE
#define DEBUG(str) do { std::cout << str << std::endl; } while( false )
#else
#define DEBUG(str) do { } while ( false )
#endif

#define FIXED	0
#define PLACED	1

#define H		0
#define V		1

#define DOWN	0
#define UP		1
#define LEFT	2
#define RIGHT	3

#define EPSILON 1e-5

#define fEq(a, b)	(fabs(a - b) < EPSILON)
#define fGe(a, b)	(a - b > 0.0)
#define fMax(a, b)	((a > b)? a : b)
#define fMin(a, b)	((a < b) ? a : b)

class Component;
class MCluster;

struct Macro
{
	string name;
	double width, height;
};

struct Bound
{
	Bound(int x1, int y1, int x2, int y2)
	{
		/* vertical boundary edge */
		if (x1 == x2) {
			dir = V;
			start = y1;
			end = y2;
			delta = y2 - y1;
			pos = x1;
			mid_pos.first = x1;
			mid_pos.second = (start + end) / 2.0;
		}
		/* horizontal boundary edge */
		else if (y1 == y2) {
			dir = H;
			start = x1;
			end = x2;
			delta = x2 - x1;
			pos = y1;
			mid_pos.first = (start + end) / 2.0;
			mid_pos.second = y1;
		}
	};

	bool getDirection() { return dir; };
	int getRelation()
	{
		if (dir == V && delta > 0) { return LEFT; }
		else if (dir == V && delta < 0) { return RIGHT; }
		else if (dir == H && delta > 0) { return UP; }
		else if (dir == H && delta < 0) { return DOWN; }
		else return -1;
	}
	double getPos() { return pos; }
	pair<double, double> get_mid_pos() { return mid_pos; }
	int getWidth() { return (dir == H) ? abs(delta) : 0.0; }
	int getHeight() { return (dir == V) ? abs(delta) : 0.0; }

	int start, end;
	int pos; // SweepLine's event position
	int delta;
	bool dir;
	pair<double, double> mid_pos;
};

class MCluster
{
public:
	// constructor and destructor
	MCluster(vector<Component*> comps) : _comps(comps)
	{
		// build bounding box for cluster
		buildBoundingBox();
	}
	~MCluster() {}

	// Get
	double get_cx() { return _cx; }
	double get_cy() { return _cy; }
	double get_lx() { return _cx - _width / 2; }
	double get_ly() { return _cy - _height / 2; }
	double get_ux() { return _cx + _width / 2; }
	double get_uy() { return _cy + _height / 2; }
	double getWidth() { return _width; }
	double getHeight() { return _height; }
	vector<Component*> getComps() { return _comps; }
	// Set
	void set_cx(double cx);
	void set_cy(double cy);
	void setWidth(double width) { _width = width; }
	void setHeight(double height) { _height = height; }
	void setComps(vector<Component*> comps) { _comps = comps; }
	//
	void buildBoundingBox();

private:

	//boundingBox
	double _cx; // center x
	double _cy; // center y
	double _width; // width
	double _height; //height

	vector<Component*> _comps;
};

class Component
{
public:

	Component(): inGroup(false), inCluster(false), isBoundary(false) { }
	Component(int llx, int lly, int w, int h) : _llx(llx), _lly(lly), width(w), height(h), inGroup(false), inCluster(false), isBoundary(false)
	{
		_originX = llx;
		_originY = lly;
	};
	Component(MCluster cluster) : _cx(cluster.get_cx()), _cy(cluster.get_cy()), width(cluster.getWidth()), height(cluster.getHeight()), inGroup(false), inCluster(false), isBoundary(false)
	{
		name = "Cluster";
		macro = (*cluster.getComps().begin())->macro;
		type = PLACED;
	}
	Component(Bound* b) : _boundary(b), inGroup(false), inCluster(false), type(FIXED), isBoundary(true), name("Boundary")
	{
		if (b->dir == V) {
			width = 0;
			height = abs(b->end - b->start);

			if (b->start < b->end) {
				_llx = b->pos;
				_lly = b->start;
			}
			else {
				_llx = b->pos;
				_lly = b->end;
			}
		}
		else if (b->dir == H) {
			width = abs(b->end - b->start);
			height = 0;

			if (b->start < b->end) {
				_llx = b->start;
				_lly = b->pos;
			}
			else {
				_llx = b->end;
				_lly = b->pos;
			}
		}
	};


	int get_mid_x() const { return _llx + width / 2.0; }				// get middle y
	int get_mid_y() const { return _lly + height / 2.0; }				// get middle x
	int getWidth() const	 { return width; }							// get macro's width
	int getHeight() const { return height; }							// get macro's height
	bool getType() const { return type; }								// tell the macro is PLACED or FIXED 
	int get_ll_x() const { return _llx; }								// get lower-left x
	int get_ll_y() const { return _lly; }								// get lower-left y
	int get_ur_x() const { return _llx + width; }						// get upper-right x
	int get_ur_y() const { return _lly + height; }						// get upper-right y
	int get_org_x() const { return _originX; };							// get initial position x (used for score_evaluation)
	int get_org_y() const { return _originY; };							// get initial position y (used for score_evaluation)
	double getArea() const { return (double)width * (double)height; }	// get macro's area
	string getName() { return name; }								// get macro's name
	pair<double, double> getForce() { return _force; }					// used for force-directed

	void set_cx(double cx) { _llx = cx - width / 2.0; }
	void set_cy(double cy) { _lly = cy - height / 2.0; }
	void setWidth(double w) { width = w; }
	void setHeight(double h) { width = h; }
	void setArg(int llx, int lly, int w, int h) {_llx = llx; _lly = lly; width = w; height = h; _originX = llx; _originY = lly;}
	void setIsBoundary(bool ib) { isBoundary = ib; }
	void setForce(double _cx, double _cy) { _force.first = _cx; _force.second = _cy; }
	void addForce(double _cx, double _cy) { _force.first += _cx; _force.second += _cy; }
	void expandWidth(double ex) { width += ex; _llx -= ex / 2; }
	void expandHeight(double ex) { height += ex; _lly -= ex / 2; }
	void shrinkWidth(double shrink) { width -= shrink; _llx += shrink / 2; }
	void shrinkHeight(double shrink) { height -= shrink; _lly += shrink / 2;}

	// Debug 
	void print_x_interval() { cout << _cx - 0.5 * width << "~" << _cx + 0.5 * width; }

public:

	string name;		// comp's name
	Macro* macro;		// the macro that the component belongs to
	Bound *_boundary;	// if we take boundary as a component
	int _cx, _cy;		// center x, center y
	int _llx, _lly;		// lower-left x, lower-left y
	int width, height;	// component's width and height
	bool type;			// FIXED or PLACED

	bool inGroup;		// Only for drawing
	bool inCluster;		// Only for drawing
	bool isSatisfyBufferArea;
	bool isBoundary;

	pair<double, double> _force;

	// Score evaluation
	int _originX; 	// lower-left y
	int _originY;	// lower-left x

	std::set<std::string> nameToken;	// split component's name by '/' 

};

// Buffer area compare Function
struct rightEdgeComp {	// Area buffer reservation
	bool operator()(const Component* a, const Component* b) const { return a->get_ur_x() < b->get_ur_x(); }
};
struct leftEdgeComp {	// Area buffer reservation
	bool operator()(const Component* a, const Component* b) const { return a->get_ll_x() < b->get_ll_x(); }
};

class Parser
{
public:
	Parser() : minOfWidth(INT_MAX), minOfHeight(INT_MAX), maxOfWidth(0), maxOfHeight(0), longestTokenLength(0) {};

	vector<Bound*> getBound() { return bound; }

	void readDef(string def_file, fstream& output, map<string, Macro*> &macro, vector<Component*> &comp);
	void readLef(string lef_file, map<string, Macro*> &macro);
	void readTxt(string txt_file);
	void moveCoordinateSystem(vector<Component*>& compVec);
	void recoverCoordinateSystem(vector<Component*>& compVec);
	void writeDef(fstream& output, vector<Component*>& comp);
	void load_bounding();
	
	// Score evaluation
	void score_evaluation(vector<Component*>& comp, unordered_set<Component*>& nfs);

public:
	string version;
	string designName;
	int dbuPerMicron;

	vector<pair<int, int>> dieArea;
	vector<Bound*> bound;
	int numComps;
	int _pwc;
	int _mcsbmc;
	int _baredc;
	int _wa;
	int _wb;

	/* setting for normalization */
	int minOfWidth, minOfHeight;
	int maxOfWidth, maxOfHeight;
	size_t longestTokenLength;

	pair<int, int> originDisplacement;
};

#endif // PARSER_H
