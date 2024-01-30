#ifndef SWEEPLINE_H
#define SWEEPLINE_H
#include "parser.h"

struct Event
{
	Event() {};

	Event(Component *c, bool b, int p, bool isB = false) : comp(c), begin(b), pos(p), isBound(isB) { };

	Event(Bound *b) : bound(b)
	{
		isBound = true;
		pos = b->pos;

		if (b->dir == H)
			begin = (b->delta < 0) ? false : true;
		else if (b->dir == V)
			begin = (b->delta > 0) ? false : true;
	};

	bool operator<(const Event& b) const
	{
		/*if (this->pos != b.pos)
			return b.pos < pos;
		else if (begin != b.begin)
			return this->begin;
		else if (isBound && b.isBound)
			return comp->_boundary->dir;*/
		if (b.pos == pos)
			return this->begin;
		else
			return b.pos < pos;
	}

	bool begin;
	int pos; // record the sweep line's path
	Component *comp;

	// store boundering
	bool isBound;
	Bound *bound;
};

// The interval in the interval tree 
class Node
{
public:
	Node()
	{
		height = 1;
		left = NULL;
		right = NULL;
	};

	// For detect overlap
	Node(Component *c, int type = V): comp(c), height(1), left(NULL), right(NULL)
	{
		if (type == V) {
			low = c->get_ll_x();
			high = c->get_ur_x();
			pos = c->get_mid_x(); // center x
		}
		else if (type == H) {
			low = c->get_ll_y();
			high = c->get_ur_y();
			pos = c->get_mid_y(); // center y
		}
		setMax(high);
	}

	Node(int l, int h) : height(1), left(NULL), right(NULL)
	{
		if (l <= h) {
			low = l;
			high = h;
		}
		else {
			low = h;
			high = l;
		}
		setMax(high);
		pos = (double)(low + high) / 2.0;
	}

	int getLow()			{ return low; }
	int getHigh()			{ return high; }
	int getMax()			{ return max; }
	int getHeight()			{ return height; }
	int getPos()			{ return pos; }
	int getLength()			{ return abs(high - low); }
	Node* getLeft()			{ return left; }
	Node* getRight()		{ return right; }
	Component* getComp()	{ return comp; }
	int getBegin()		{ return begin; }
	int getPathPos()		{ return _pathPos; }


	void setLow(int low)			{ this->low = low; }
	void setHigh(int high)			{ this->high = high; }
	void setMax(int max)			{ this->max = max; }
	void setHeight(int height)		{ this->height = height; }
	void setPos(int pos)			{ this->pos = pos; }
	void setLeft(Node* left)		{ this->left = left; }
	void setRight(Node* right)		{ this->right = right; }
	void setComp(Component* comp)	{ this->comp = comp; }
	void setBegin(int begin)		{ this->begin = begin; }
	void setPathPos(int pathPos)	{ this->_pathPos = pathPos; }

public:

	int low, high;
	int max;
	int height;		// tree height
	int pos;		// center
	int _pathPos; 	// event pos
	Node *left, *right;

	Component *comp;
	Component* _leftComp, *_rightComp;	// the left and right point of interval


	// For Free Space
	int begin;
};


// Data structure: AVL tree
class IntervalTree
{
public:
	IntervalTree() { root = NULL; };

	void setMinSpace(int minSpace)	{ _minSpace = minSpace; }

	bool ifOverlap(Node* i, Node* j, const int minSpace);
	int height(Node *N);
	int maxHeight(Node* node);
	int maxValue(Node* node);
	void updateMaxVaue(Node *node);
	Node* minValueNode(Node *node);
	int getBalance(Node* node);

	Node* rightRotate(Node* _cy);
	Node *leftRotate(Node *_cx);

	// Overlapping Operation
	Node* insert(Node *current, Node *add);
	Node* updateTreeBalance(Node* current);
	Node* deleteNode(Node *root, Node *node);
	void search_overlap(vector<Component*>& o_comp, Node *root, Node *n, const int minSpace);

	void search_overlap_for_overlap_group(vector<Component*>& o_comp, Node *root, Node *n);
	bool ifOverlap_for_overlap_group(Node* i, Node* j);

	// Cluster Component inside macro
	bool ifCluster(Node* i, Node* j);
	void search_cluster(vector<Component*>& _clusterComp, Node *root, Node *n);

	// Free Space Operation
	bool ifFreeSpaceOverlap(Node* i, Node* j, bool direction);
	Node* get_interval(Node* root, Node *n, bool direction);
	Node* find_left_adjacent_interval(Node* root, int point);
	Node* find_right_adjacent_interval(Node* root, int point);

	// Debug
	void print_infix(Node* root);

public:
	Node* root;

	int _minSpace;
};

#endif // !SWEEPLINE_H