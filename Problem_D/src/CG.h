#ifndef CG_H
#define CG_H
#include "parser.h"
#include <time.h>

namespace cg {

    class Point{
    public:
        Point() {};
        Point(Component* comp);

    public:
        Component *_comp;

        int _lm, _rm;
        int _dm, _um;

        size_t _no;     // index of the point

        std::vector<pair<size_t, int>> _left, _right;
        std::vector<pair<size_t, int>> _down, _up;
        Point *_criticalHParent, *_criticalVParent;
        Point *_criticalHInverseParent, *_criticalVInverseParent;
    };

    class CG {
    public:
        CG(vector<Component*>& comp, vector<Component*>& boundaryComp, Parser& parser) 
        : _compVec(comp), _boundaryComp(boundaryComp), _parser(parser)
        {
            _minSpace = parser._mcsbmc * parser.dbuPerMicron;
            _pwc = parser._pwc * parser.dbuPerMicron;
            _designLeft = parser.minOfWidth;
            _designRight = parser.maxOfWidth;
            _designDown = parser.minOfHeight;
            _designUp = parser.maxOfHeight;
        };
        ~CG() 
        {
            
        }

        /**/
        void cal_leftMost();
	    void cal_rightMost();
	    void cal_downMost();
	    void cal_upMost();
        
        void initialize();
        void buildGraph();
        void addBoundaryPoint();

        int getHLongestPath(Point*, Point*&);
        int getVLongestPath(Point*, Point*&);
        int getHInverseLongestPath(Point*, Point*&);
        int getVInverseLongestPath(Point*, Point*&);
        int getRelation(Point*, Point*);

        /* operation */
        void deleteEdge(vector<pair<size_t, int>>&, size_t);
        void deleteEdge(vector<pair<size_t, int>>&, Point*);
        void deletePoint(Point*, bool);
        void swap(Point*, Point*);
        void move(Point*, Point*, bool);

        /* solver */
        void solveLPbyCG();
        void solveLPbyP2P();

        void legalizeCriticalPath();
        void legalize();
        void optBufferArea();
        void legalizeBufferConstraint(vector<Component*>&);
        void leaveSpaceForBufferArea(vector<Component*>&);
        bool optimizeBufferArea(Point*, Point*, bool);
        void optDeadSpace();
        void stickToBoundary();

    public:
        Parser& _parser;
        std::vector<Component*>& _compVec;
        std::vector<Point*> _pointVec;
        std::vector<Component*>& _boundaryComp; // boundary tile

        Point *_hsource, *_hsink;
        Point *_vsource, *_vsink;

        /* design bounds */
        int _designLeft, _designRight;
        int _designDown, _designUp;

        int _minSpace;  // minimum_channel_spacing_between_macros_constraint
        int _pwc;       // powerplan_width_constraint

        /* Debug */
        vector<Component*> deletedPoint;
        vector<Component*> movedMacro;
        vector<Component*> swappedPoint;

    public:
        /* static member variable*/
	    static bool startBufferAreaOpt;
        static bool startBufferAreaLeg;
        static vector<Component*> lastDeletedPoint;
    };

}
#endif