#ifndef LEGALIZER_H 
#define LEGALIZER_H

#include "tile.h"
#include "parser.h"

using namespace CornerStitch;

class Legalizer{
    public:
        Legalizer(vector<Component*>& compVec, Plane* plane, Rectangle boundingRect, int minSpace) :
            _compVec(compVec), _plane(plane), _boundingRect(boundingRect), _minSpace(minSpace){
        };

        void legalize();

        Rectangle getBoundingRect() { return _boundingRect; }
        Plane* getPlane() { return _plane; }
        int getMinSpace() { return _minSpace; }

        void plot();

        void buildBoundingRect();
        void buildBoundaryTile();

    private:
        Rectangle                       _boundingRect;  // the bounding box of entire design
        vector<Component*>&             _compVec;       // vector reference all macros in the design
        Plane*                          _plane;         // plane for the entire design
        int                             _minSpace;      //

};


enum class PosType { Legal, Fill, Invalid };

class TileFindLegalOp : public TileOp{
    public:
        TileFindLegalOp(Legalizer& legalizer, 
                Component* comp, 
                Rectangle& regionRect, 
                CornerStitch::Point& bestPoint, 
                PosType& findResult)
            : _legalizer(legalizer), _comp(comp), _regionRect(regionRect), 
            _bestPoint(bestPoint), _findResult(findResult), _bestCost(INF){
        }
        
        virtual ~TileFindLegalOp() {}

        virtual int operator() (Tile* tile);

    private:
        void packEval(Tile* tile, Rectangle*);
        PosType checkPosType(Tile* tile, Rectangle*);
        double evalCost(int x, int y);

        Legalizer&                  _legalizer;         //
        Component*                  _comp;              //
        Rectangle&                  _regionRect;        // the region macro can't exceed
        CornerStitch::Point&        _bestPoint;         // position of the current best cost
        PosType&                    _findResult;        // found at least one legal or fill position
        double                      _bestCost;          // the current best cost
        

};

class TileDrawOp : public TileOp{
    public:
        TileDrawOp(fstream& outgraph, Rectangle& boundingRect, int minSpace) 
            : _outgraph(outgraph), _boundingRect(boundingRect), _minSpace(minSpace), _index(1){
        }

        virtual ~TileDrawOp() {}

        virtual int operator() (Tile* tile);

    private:
        fstream&    _outgraph;
        Rectangle&  _boundingRect;
        int         _minSpace;
        int         _index;
};

#endif
