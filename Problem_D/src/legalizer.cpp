#include "legalizer.h"
#include <algorithm>

static bool groupCompare(vector<Component*> a, vector<Component*> b){
    return a[0]->getArea() > b[0]->getArea();    
}

static bool areaCompare(Component* a, Component* b){
    return a->getArea() > b->getArea();
}

static bool macroCompare(Component* a, Component* b){
    return a->getArea() > b->getArea();
}

/*
 *
 * convert Component object to Rectangle object.
 *
 */
static void component2Rect(Component* comp, Rectangle& rect){
    rect.setLeft(comp->get_ll_x());
    rect.setBottom(comp->get_ll_y());
    rect.setRight(comp->get_ur_x());
    rect.setTop(comp->get_ur_y());
}

/*
 * */
void evalRect4Group(vector<Component*>& group, Rectangle &rect){
    component2Rect(group[0], rect);

    for(size_t i = 1; i < group.size(); ++i){
        if(group[i]->get_ll_x() < rect.getLeft())
            rect.setLeft(group[i]->get_ll_x());
        if(group[i]->get_ll_y() < rect.getBottom())
            rect.setBottom(group[i]->get_ll_y());
        if(group[i]->get_ur_x() > rect.getRight())
            rect.setRight(group[i]->get_ur_x());
        if(group[i]->get_ur_y() > rect.getTop())
            rect.setTop(group[i]->get_ur_y());
    }
}


/*
 *
 *
 *
 */
PosType TileFindLegalOp::checkPosType(Tile* tile, Rectangle* macroRect){
    int minSpace = _legalizer.getMinSpace();

    macroRect->scaleUR(-minSpace);
    if(CornerStitch::searchAreaAllSpace(tile, _legalizer.getPlane(), macroRect) == false)
        return PosType::Invalid;

    macroRect->scaleUR(minSpace);
    if(CornerStitch::searchAreaAllSpace(tile, _legalizer.getPlane(), macroRect))
        return PosType::Legal;
    
    if(CornerStitch::searchAreaNoMacro(tile, _legalizer.getPlane(), macroRect))
        return PosType::Fill;
     
    return PosType::Invalid;
}


/*
 *
 *
 *
 */
void TileFindLegalOp::packEval(Tile* tile, Rectangle* macroRect){
    PosType posType = checkPosType(tile, macroRect);
    if(posType != PosType::Invalid){
        double cost = evalCost(macroRect->getLeft(), macroRect->getBottom());

        if(_bestCost > cost){
            _findResult = posType;
            _bestCost = cost;
            _bestPoint._x = macroRect->getLeft();
            _bestPoint._y = macroRect->getBottom();
        }
    }
}



/*
 * 
 *
 *
 */
int TileFindLegalOp::operator() (Tile* tile){
    /* if the type of tile isn't Space, nothing to do */
    if(tile->_tileType != TileType::Space)
        return 0;

    int minSpace = _legalizer.getMinSpace();

    /* spaceRect represent the region macro can place in this space tile */
    Rectangle spaceRect;
    spaceRect.tile2Rect(tile);
    spaceRect.clip(_regionRect);
    
    int macroWidth = _comp->getWidth() + minSpace;
    int macroHeight = _comp->getHeight() + minSpace;

    /* candidate column and row number */
    Rectangle macroRect;
    int candCol = spaceRect.getWidth() / macroWidth;
    int candRow = spaceRect.getHeight() / macroHeight;
    //cout << "candidate: " << candRow << " " << candCol << endl;

    /* place macro from lower left corner of space tile */
    for(size_t i = 0; i <= candRow; ++i){
        for(size_t j = 0; j <= candCol; ++j){
            macroRect.set4Side(spaceRect.getLeft() + j * macroWidth,
                    spaceRect.getBottom() + i * macroHeight,
                    spaceRect.getLeft() + (j + 1) * macroWidth,
                    spaceRect.getBottom() + (i + 1) * macroHeight);

            if(macroRect.getRight() <= _regionRect.getRight() && macroRect.getTop() <= _regionRect.getTop()){
                packEval(tile, &macroRect);
            }
        }
    }

    /* place macro from lower right corner of space tile*/
    for(size_t i = 0; i <= candRow; ++i){
        for(size_t j = 0; j <= candCol; ++j){
            macroRect.set4Side(spaceRect.getRight() - (j + 1) * macroWidth,
                    spaceRect.getBottom() + i * macroHeight,
                    spaceRect.getRight() - j * macroWidth,
                    spaceRect.getBottom() + (i + 1) * macroHeight);

            if(macroRect.getLeft() >= _regionRect.getLeft() && macroRect.getTop() <= _regionRect.getTop()){
                packEval(tile, &macroRect);
            }
        }
    }

    /* place macro at upper left corner of space tile*/ 
    for(size_t i = 0; i <= candRow; ++i){
        for(size_t j = 0; j <= candCol; ++j){
            macroRect.set4Side(spaceRect.getLeft() + j * macroWidth,
                    spaceRect.getTop() - (i + 1) * macroHeight,
                    spaceRect.getLeft() + (j + 1) * macroWidth,
                    spaceRect.getTop() - i * macroHeight);

            if(macroRect.getRight() <= _regionRect.getRight() && macroRect.getBottom() >= _regionRect.getBottom()){
                packEval(tile, &macroRect);
            }
        }
    }

    /* place macro at upper right corner of space tile*/
    for(size_t i = 0; i <= candRow; ++i){
        for(size_t j = 0; j <= candCol; ++j){
            macroRect.set4Side(spaceRect.getRight() - (j + 1) * macroWidth,
                    spaceRect.getTop() - (i + 1) * macroHeight,
                    spaceRect.getRight() - j * macroWidth,
                    spaceRect.getTop() - i * macroHeight);

            if(macroRect.getLeft() >= _regionRect.getLeft() && macroRect.getBottom() >= _regionRect.getBottom()){
                packEval(tile, &macroRect);
            }
        }
    }
    
    return 0;
}


/*
 * 
 * legalize the macro placement by placing marco one by one.
 *
 */
void Legalizer::legalize(){

    Rectangle rect;
    int halfMinSpace = std::ceil((double)_minSpace / 2.0);
    /* place pre-placed macros into corner stitching tile plane */
    for(size_t i = 0; i < _compVec.size(); ++i){
        // place each pre-placed macro 
        if(_compVec[i]->getType() == FIXED){
            component2Rect(_compVec[i], rect);
            rect.scaleUR(_minSpace);
            //rect.scale(halfMinSpace);
            rect.clip(_boundingRect);

            if(CornerStitch::searchAreaAllSpace(NULL, _plane, &rect))
                Tile* tile = CornerStitch::insertTile(&rect, _plane);
            else
                CornerStitch::fillArea(NULL, _plane, &rect);
            
        }
    }

    std::sort(_compVec.begin(), _compVec.end(), macroCompare);
   
    /* place movable macros into corner stitching tile plane */
    for(size_t i = 0; i < _compVec.size(); ++i){
        if(_compVec[i]->getType() == PLACED){
            component2Rect(_compVec[i], rect);
            rect.scaleUR(_minSpace);
            //rect.scale(halfMinSpace);
            rect.clip(_boundingRect);

            if(CornerStitch::searchAreaAllSpace(NULL, _plane, &rect)){
                CornerStitch::insertTile(&rect, _plane);
            }
            else{
                PosType findResult = PosType::Invalid;
                CornerStitch::Point bestPoint;
                TileOp* tileOp = new TileFindLegalOp(*this, _compVec[i], _boundingRect, bestPoint, findResult);
                CornerStitch::searchArea(NULL, _plane, &_boundingRect, tileOp);
                delete tileOp;

                if(findResult != PosType::Invalid){
                    Rectangle macroRect(bestPoint._x, 
                            bestPoint._y,
                            bestPoint._x + _compVec[i]->getWidth() + _minSpace,
                            bestPoint._y + _compVec[i]->getHeight() + _minSpace);
                    if(findResult == PosType::Legal){
                        cout << "insert macro\n";
                        CornerStitch::insertTile(&macroRect, _plane);
                    }
                    else{
                        cout << "fill macro\n";
                        bestPoint.print();
                        CornerStitch::fillArea(NULL, _plane, &macroRect);
                    }
                    
                    _compVec[i]->_llx = bestPoint._x;
                    _compVec[i]->_lly = bestPoint._y;
                }
                else
                    cout << "the entire design no space to put macro " << _compVec[i]->getName() << endl;
            }
        }
    }

    /* sort macro with its area 
    for(size_t i = 0; i < _overlapGroup.size(); ++i)
        std::sort(_overlapGroup[i].begin(), _overlapGroup[i].end(), areaCompare);

    std::sort(_overlapGroup.begin(), _overlapGroup.end(), groupCompare);
    */ 
}

/*
 * */
double TileFindLegalOp::evalCost(int x, int y){
    return abs(_comp->get_org_x() - x) + abs(_comp->get_org_y() - y);
}

int TileDrawOp::operator()(Tile* tile){
    int left = (tile->getLeft() >= _boundingRect.getLeft()) ? tile->getLeft() : _boundingRect.getLeft(); 
    int right = (tile->getRight() <= _boundingRect.getRight()) ? tile->getRight() : _boundingRect.getRight(); 
    int bottom = (tile->getBottom() >= _boundingRect.getBottom()) ? tile->getBottom() : _boundingRect.getBottom(); 
    int top = (tile->getTop() <= _boundingRect.getTop()) ? tile->getTop() : _boundingRect.getTop();

    _outgraph << "set object " << _index << " rect from " << left << "," << bottom 
        << " to " << right << "," << top; 
    if(tile->getTileType() == TileType::Invalid)
        _outgraph << " fc rgb 'greenyellow'";
    else if(tile->getTileType() == TileType::Boundary)
        _outgraph << " fc rgb 'gold'";
    else if(tile->getTileType() == TileType::Macro)
        _outgraph << " fc rgb '#ee3300'";
    else
        _outgraph << " fc rgb '#0f0f0f'";
    _outgraph << " fs transparent solid 0.4 border lc rgb '#010101'\n";

    ++_index;

    /*if(tile->getTileType() == TileType::Macro){
        _outgraph << "set object " << _index << " rect from " << left << "," << bottom 
            << " to " << right - _minSpace  << "," << top - _minSpace << "fc rgb '#aaee00' fs transparent solid 0.4 noborder lc rgb '#010101'\n"; 

    }

    ++_index;
    */
    return 0;
}

void Legalizer::plot(){
    fstream outgraph("legalizer.gp", ios::out);

    outgraph << "reset\n";
    outgraph << "set tics\n";
    outgraph << "unset key\n";
    outgraph << "set title \"The result of Corner Stitching\"\n";
    
    TileOp* tileOp = new TileDrawOp(outgraph, _boundingRect, _minSpace);
    CornerStitch::searchArea(NULL, _plane, &_boundingRect, tileOp);
    delete tileOp;
    
    //_boundingRect.scale(_boundingRect.getWidth() * 0.2);

    outgraph << "set style line 1 lc rgb \"red\" lw 3\n";
    outgraph << "set border ls 1\n";
    outgraph << "set terminal png\n";
    outgraph << "set output \"corner_stitch.png\"\n";
    outgraph << "plot [" << _boundingRect.getLeft() << ":" << _boundingRect.getRight() << "][" << _boundingRect.getBottom() << ":" 
        << _boundingRect.getTop() << "] \'line\' w l lt 2 lw 1\n";
    outgraph << "set terminal x11 persist\n";
    outgraph << "replot\n";
    outgraph << "exit";
    outgraph.close();
	
    system("gnuplot legalizer.gp");
}











