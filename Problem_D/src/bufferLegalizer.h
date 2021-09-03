#ifndef BUFFERLEGALIZER_H
#define BUFFERLEGALIZER_H
#include "parser.h"
#include "FreeSpace.h"
#include "CG.h"

namespace buffer {

    class BufferLegalizer {
    public:
        BufferLegalizer(vector<Bound*>& boundary, vector<Component*>& compVec, vector<Component*>& boundaryComp, Parser& parser) :
        _boundary(boundary), _compVec(compVec), _boundaryComp(boundaryComp), _parser(parser) {};

        vector<Component*> getIllegal() { return _illegal; }
        vector<Component*> getDeletedComp() { return _deletedComp; }
        unordered_set<Component*> getFreeSpace() { return _freeSpace; }
        unordered_set<Component*> getDeadSpace() { return _deadSpace; }


        bool ifOverlap(Component* comp, Component* fs);
        void findIllegal();
        void legalize();
    public:
        Parser& _parser;
        vector<Component*>& _compVec;
        vector<Bound*>&     _boundary;
        vector<Component*>& _boundaryComp;

        unordered_set<Component*> _freeSpace;
        unordered_set<Component*> _deadSpace;
        vector<Component*>  _deletedComp;
        vector<Component*>  _illegal;
    };

};

#endif