#include "bufferLegalizer.h"
using namespace buffer;

/*
*
* find if comp and freespace overlap
*
*/
bool BufferLegalizer::ifOverlap(Component* comp, Component* fs)
{   
    int extend = _parser._baredc * _parser.dbuPerMicron; //buffer_area_reservation_extended_distance_constraint
    int rec1[4] = { comp->get_ll_x() - extend, comp->get_ll_y() - extend,   // lower-left xy
                    comp->get_ur_x() + extend, comp->get_ur_y() +extend };  // upper-right xy
	int rec2[4] = { fs->get_ll_x(), fs->get_ll_y(), fs->get_ur_x(), fs->get_ur_y() };

    return (rec1[0] < rec2[2] && rec2[0] < rec1[2] && rec1[1] < rec2[3] && rec2[1] < rec1[3]) ? true : false;
}

/*
*
*
* find the macros who violate buffer area constraint
*
*/
void BufferLegalizer::findIllegal()
{
    _illegal.clear();
    // each macro
    for (vector<Component*>::iterator compIter = _compVec.begin(); compIter != _compVec.end(); ++compIter) {
        Component* comp = *compIter;

		if (std::find(_deletedComp.begin(), _deletedComp.end(), comp) != _deletedComp.end())
			continue;

        bool legal = false;
        // each free space
        for (unordered_set<Component*>::iterator fsIter = _freeSpace.begin(); fsIter != _freeSpace.end(); ++fsIter) {
            Component* fs = *fsIter;
            /* leggal*/
            if (ifOverlap(comp, fs)) {
                legal = true;
                break;
            }
        }
        if (!legal)
            _illegal.push_back(comp);
    }
}

/*
*
* all macros excluding deleted macros meet buffer area constraint
*
*/
void BufferLegalizer::legalize()
{
    /* find free space */
    FreeSpace fs(_compVec, _boundary, _parser, _deletedComp);
	fs.findFreeSpace();
	_freeSpace = fs.getFreeSpace();
	_deadSpace = fs.getNotFreeSpace();

    /* find the macros who violate buffer area constraint*/
    findIllegal();

    /* loop until black macros are all meet the buffer area constraint*/
    cg::CG cg(_compVec, _boundaryComp, _parser);

	fstream file("delete.txt", ios::out);

    while(!_illegal.empty()) {

		//file << count << "'s BufferLegalizer" << endl;
		
        cg.legalizeBufferConstraint(_illegal);

        _deletedComp = cg.deletedPoint;

		/*file << "_deletedComp size: " << _deletedComp.size() << endl;
		cout << "_deletedComp size: " << _deletedComp.size() << endl;
		for (auto i = _deletedComp.begin(); i != _deletedComp.end(); ++i) {
			file << (*i)->name << endl;
			cout << (*i)->name << endl;
		}*/

        fs._deletedComp = _deletedComp;
		/*file << "fs._deletedComp size: " << fs._deletedComp.size() << endl;
		cout << "fs._deletedComp size: " << fs._deletedComp.size() << endl;*/

        fs.findFreeSpace();
        _freeSpace = fs.getFreeSpace();
	    _deadSpace = fs.getNotFreeSpace();
        findIllegal();

		/*file << "Illegal size: " << _illegal.size() << endl;
		for (auto i = _illegal.begin(); i != _illegal.end(); ++i) {
			file << (*i)->name << endl;
		}*/
    }
}