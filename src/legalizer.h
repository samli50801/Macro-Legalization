#ifndef LEGALIZER_H 
#define LEGALIZER_H

#include "tile.h"
#include "parser.h"

using namespace CornerStitch;

class Legalizer{
    public:
        Legalizer(vector<Component*>& compVec, 
                Plane* plane, 
                Rectangle boundingRect, 
                Parser& parser) :
            _compVec(compVec), 
            _plane(plane), 
            _boundingRect(boundingRect),
            _needCheckBuffer(false) {
                _minSpace = parser._mcsbmc * parser.dbuPerMicron;
                _pwc = parser._pwc * parser.dbuPerMicron;
                _baredc = parser._baredc * parser.dbuPerMicron;
        };


        Rectangle getBoundingRect() { return _boundingRect; }
        Plane* getPlane() { return _plane; }
        Plane* getBufferPlane() { return _bufferPlane; }
        void buildBufferPlane(unordered_set<Component*>& freeSpaceVec, Plane* bufferPlane);

        int getMCSBMC() { return _minSpace; }
        int getPWC() { return _pwc; }
        int getBAREDC() { return _baredc; }
        
        void setPlane(Plane* plane) { _plane = plane; }
        void needCheckBuffer() { _needCheckBuffer = true; }
        bool getNeedCheckBuffer() { return _needCheckBuffer; }

        bool checkMacroPos(Rectangle macroRect);
        bool checkMacroBuffer(Rectangle macroRect);
        bool findLegalBufferPos(Rectangle macroRect, Rectangle& bufferRect);
        bool findLegalBufferPosFromPlane(Rectangle macroRect);
        bool findLegalMacroPos(Component* macroComp, Rectangle macroRect);
        bool findAndPlace(Component* macroComp);
        bool placeMacro(Rectangle macroRect);
        bool placeBuffer(Rectangle bufferRect);

        void legalize(vector<Component*>& nonPlacedVec);
        void plot();
        void bufferPlot();

    private:

        Rectangle                       _boundingRect;  // the bounding box of entire design
        vector<Component*>&             _compVec;       // vector reference all macros in the design
        Plane*                          _plane;         // plane for the entire design
        Plane*                          _bufferPlane;   // buffer plane for the entire design
        bool                            _needCheckBuffer;   // the legalizer need to check buffer area constraint or not

        int                             _minSpace;
        int                             _pwc;
        int                             _baredc;
};


enum class FindType { Legal, BufferInsert, Invalid };

class TileFindLegalOp : public TileOp{
    public:
        TileFindLegalOp(Legalizer& legalizer, 
                Rectangle macroRect,
                Rectangle& bestMacroRect,
                Rectangle& bufferRect,
                Rectangle searchRect, 
                FindType& findResult)
            : _legalizer(legalizer), _macroRect(macroRect), _bestMacroRect(bestMacroRect),
            _bufferRect(bufferRect), _searchRect(searchRect), _findResult(findResult), _bestCost(INF){
        }
        
        virtual ~TileFindLegalOp() {}

        virtual int operator() (Tile* tile);

    private:
        void packEval(Tile* tile, Rectangle);
        double evalCost(int x, int y);

        Legalizer&                  _legalizer;         //
        Rectangle                   _macroRect;         // macro original position
        Rectangle&                  _bestMacroRect;     // macro original position
        Rectangle&                  _bufferRect;        // insert buffer position
        Rectangle                   _searchRect;        // the region macro can't exceed
        FindType&                   _findResult;        // legal, buffer insert, or not found
        double                      _bestCost;          // the current best cost
        

};





class TileFindBufferOp : public TileOp{
    public:
        TileFindBufferOp(Legalizer& legalizer, 
                Rectangle macroRect,
                CornerStitch::Point& bestPoint,
                bool& findResult) 
            :_legalizer(legalizer), _macroRect(macroRect),
            _bestPoint(bestPoint), _findResult(findResult), _bestCost(INF){
                _searchRect = _macroRect;
                _searchRect.scale(_legalizer.getBAREDC());
            }

        virtual ~TileFindBufferOp() {}

        virtual int operator() (Tile* tile);

    private:
        double evalCost(int x, int y){
            return abs(x - _macroRect.getLeft()) + abs(y - _macroRect.getRight());
        }

        Legalizer&              _legalizer;
        Rectangle               _macroRect;
        Rectangle               _searchRect;
        CornerStitch::Point&    _bestPoint;
        bool&                   _findResult;
        double                  _bestCost;
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
