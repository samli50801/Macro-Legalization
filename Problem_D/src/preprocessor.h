#ifndef PREPROCESSOR_H
#define PREPROCESSOR_H

#include "tile.h"
#include "parser.h"
#include "SweepLine.h"

using namespace CornerStitch;

class Preprocessor{
    public:
        Preprocessor(vector<Bound*>& boundVec, Parser& parser) : _boundVec(boundVec), _parser(parser) {}

        Rectangle getBoundingRect() { return _boundingRect; }
        Plane* getPlane() { return _plane; }
        vector<Component*>& getBoundaryComp() { return _boundaryComp; }

        void run();

    private:
        void buildBoundingRect();
        void buildBoundaryTile();

        /* use sweep line to generate boundary component*/
        void insertBoundaryCompnent(int lower_x, int lower_y, int width, int height, vector<Component*>&);
        void buildInsideBoundaryComponent(vector<Component*> &);
	    void buildOutsideBoundaryComponent(vector<Component*> &);
	    void buildBoundaryComponent();  

        /* preprocessor member variables */
        Parser&             _parser;        // parser reference
        vector<Bound*>&     _boundVec;      // the boundary edges of the design
        Rectangle           _boundingRect;  // the bounding box of overall design
        Plane*              _plane;         // the plane of the overall design
        vector<Component*>  _boundaryComp;  // the boundary components for next stage

};

#endif
