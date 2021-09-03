#ifndef TCG_H
#define TCG_H
#include <float.h>
#include <unordered_map>
#include "SweepLine.h"

extern vector<Bound*> bound;

struct Point
{
public:
	
	// Constructor for normal point
	Point(Component *c) : comp(c) 
	{
		if (c->getType() == FIXED) {
			_lm = c->get_mid_x();
			_rm = c->get_mid_x();
			_dm = c->get_mid_y();
			_um = c->get_mid_y();
		}
		else {
			_lm = -1.0;
			_rm = DBL_MAX;
			_dm = -1.0;
			_um = DBL_MAX;
		}
	};
	// Constructor for sink and terminal
	Point()
	{
		Component* _c = new Component(0, 0, 0.0, 0.0);
		_c->name = "pseudoPoint";
		_c->type = PLACED; // In order to record the critical path length
		comp = _c;
	}

	// Access operation
	double get_cx() { return comp->get_mid_x(); }
	double get_cy() { return comp->get_mid_y(); }
	double getWidth() { return comp->getWidth(); }
	double getHeight() { return comp->getHeight(); }
	vector<pair<Point*, double>>& getPre() { return left; }
	vector<pair<Point*, double>>& getNext() { return right; }
	Component* getComp() { return comp; }
	bool getType() { return getComp()->type; }
	string getName() { return comp->getName(); }

	void setLeftMost(double lm) { _lm = lm; }
	void setRightMost(double rm) { _rm = rm; }
	void setDownMost(double dm) { _dm = dm; }
	void setUpMost(double um) { _um = um; }

	// Data
	Component *comp;

	// Horizontal CG
	vector<pair<Point*, double>> left, right;	// Horizontal CG's left link and right link
	double _lm, _rm;							// leftMost and rightMost

	// Vertical CG
	vector<pair<Point*, double>> up, down;		// Vertical CG's up link and down link
	double _dm, _um;							// downMost and upMost
};

class TCG
{
public:
	TCG() 
	{
		_hSource = new Point();
		_hSink = new Point();

		_vSource = new Point();
		_vSink = new Point();

		limit[0] = -DBL_MAX; // DOWN
		limit[1] = DBL_MAX; // UP
		limit[2] = -DBL_MAX; // LEFT
		limit[3] = DBL_MAX ; // RIGHT

		leftest = NULL;
		lowest = NULL;
		rightest = NULL; 
		highest = NULL;
	};

	~TCG()
	{
		delete _hSource;	_hSource = NULL;
		delete _hSink;	_hSink = NULL;
		delete _vSource;	_vSource = NULL;
		delete _vSink;	_vSink = NULL;
	}

	Point* getHSource() { return _hSource; }
	Point* getVSource() { return _vSource; }
	Component* getLeftestBlock() { return leftest; }
	Component* getLowestBlock()  { return lowest; }
	Component* getRightestBlock() { return rightest; }
	Component* getHighestBlock() { return highest; }
	vector<Component*>& get_ogroup() { return _ogroup; }
	void set_ogroup(vector<Component*> ogroup) { _ogroup = ogroup; }
	void setMinSpace(double minSpace) { _minSpace = minSpace; }

	void insertPoint(Point *root, Point *add);
	Point* newPoint(Component* comp);
	// Build constraint graph including horizontal and vertical
	void buildTCG(); 
	void cal_leftMost(Point* root, double leftMost);
	void cal_rightMost(Point* root, double rightMost);
	void cal_downMost(Point* root, double downMost);
	void cal_upMost(Point* root, double upMost);

	// calculate valid position 
	// (leftMost, rightMost)
	// (downMost, upMost)
	void cal_validPos();
	void findNearBoundary();
	void decideXPosition(Point* current, double updatedLeftMost);
	void decideYPosition(Point* current, double updatedRightMost);

	void optimizePosition();

	void printHTCG(Point* root);
	void printVTCG(Point* root);

private:
	vector<Component*> _ogroup;					// Overlapped group
	Point *_hSource, *_hSink;					// Horizontal CG's source and sink (pseudopoint)
	Point *_vSource, *_vSink;					// Vertical CG's source and sink (pseudopoint)
	unordered_map<Component*, Point*> _pnts;	// Record all points

	double _minSpace;

	Component *leftest, *lowest;				// Record the leftest and lowest block in one CG(overlapped group) 
	Component *rightest, *highest;				// Record the rightest and highest block in one CG(overlapped group) 

	double limit[4];

};

#endif // !TCG_H

