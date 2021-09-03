#ifndef CLUSTER_H
#define CLUSTER_H
#include <queue>
#include "parser.h"
#include "CG.h"

namespace bc {
	
	class Group {
	public:
		Group() : _valid(true) {};
		~Group() {};

		int _cgx, _cgy;							// center of gravity x/y
		std::vector<Component*> _members;		// the members in the group
		Macro* _macro;

		bool _valid;

		/* score function setting */
		std::set<std::string> 	_token;			// collect all common tokens from members in one group
		double 					_totalArea;		// the total area of members
	};

	typedef pair<pair<Group*, Group*>, double> Tuple; // (group1Index, group2Index, score)

	/* sort by cost function */
	struct CompareByCost {
  		bool operator()(const Tuple& t1, const Tuple& t2) const {
    		return t1.second > t2.second;
  		}
	};

	class BestChoiceCluster {
	public:
		BestChoiceCluster(vector<Component*>& compVec, Parser& parser) : _compVec(compVec), _parser(parser) {};

		void initGroupForEachComp();
		void initPriorityQueue();
		double evalCost(Group *g1, Group *g2);
		Group* getClostestGroup(Group*);
		Group* clusterGroup(Group *g1, Group *g2);
		bool satisfyHardConstraints(Group *g1, Group *g2);
		void printGroups();
		void printPriorityQueue();
		void bestChoiceClustering();

		std::vector<Group*> getGroups()				{ return _groups; };
		void setDesignWidth(int designWidth)		{ _designWidth = designWidth; }
		void setDesignHeight(int designHeight)		{ _designHeight = designHeight; }
		void setLongestTokenLength(int longestTokenLength)	{ _longestTokenLength = longestTokenLength; }

	private:
		std::vector<Component*>&		_compVec;	// vector reference all macros in the design
		std::vector<Group*> 			_groups;	// record the current groups
		std::priority_queue<Tuple, 
		vector<Tuple>, CompareByCost>	_PQ;		// prioruty queue to record tuple
		Parser& _parser;

		/* setting for normalization */
		int 	_designWidth, _designHeight;
		double	_designArea;
		size_t 	_longestTokenLength;
	};
}
#endif