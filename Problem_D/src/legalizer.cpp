#include "legalizer.h"
#include <algorithm>
#include <unordered_set>

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
void TileFindLegalOp::packEval(Tile* tile, Rectangle macroRect){
    FindType findType = FindType::Invalid;
    bool posLegal = _legalizer.checkMacroPos(macroRect);
    bool hasBuffer = _legalizer.getNeedCheckBuffer() ? _legalizer.checkMacroBuffer(macroRect) : true;
    bool canInsertBuffer = false;
    Rectangle bufferRect;

    if(hasBuffer == false)
        canInsertBuffer = _legalizer.findLegalBufferPos(macroRect, bufferRect);

    if(posLegal && hasBuffer)
        findType = FindType::Legal;
    else if(posLegal && canInsertBuffer)
        findType = FindType::BufferInsert;

    if(findType != FindType::Invalid){
        double cost = evalCost(macroRect.getLeft(), macroRect.getBottom());

        if(_bestCost > cost){
            _findResult = findType;
            _bestCost = cost;
            _bufferRect = bufferRect;
            _bestMacroRect = macroRect;
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

    int minSpace = _legalizer.getMCSBMC();
    int powerplanWidth = _legalizer.getPWC();

    /* spaceRect represent the region macro can place in this space tile */
    Rectangle spaceRect;
    spaceRect.tile2Rect(tile);
    spaceRect.clip(_searchRect);
    
    int macroWidth = _macroRect.getWidth();
    int macroHeight = _macroRect.getHeight();
    int pitchH = macroWidth + minSpace;
    int pitchV = macroHeight + minSpace;

    /* candidate column and row number */
    Rectangle macroRect;
    int candCol = spaceRect.getWidth() / macroWidth;
    int candRow = spaceRect.getHeight() / macroHeight;
    //cout << "candidate: " << candRow << " " << candCol << endl;

    /* place macro from lower left corner of space tile */
    for(size_t i = 0; i <= candRow; ++i){
        for(size_t j = 0; j <= candCol; ++j){
            macroRect.set4Side(spaceRect.getLeft() + j * pitchH,
                    spaceRect.getBottom() + i * pitchV,
                    spaceRect.getLeft() + j * pitchH + macroWidth,
                    spaceRect.getBottom() + i * pitchV + macroHeight);

            if(macroRect.getRight() <= _searchRect.getRight() && macroRect.getTop() <= _searchRect.getTop()){
                packEval(tile, macroRect);
            }
        }
    }

    /* place macro at lower right corner of space tile*/
    for(size_t i = 0; i <= 1; ++i){
        macroRect.set4Side(spaceRect.getRight() - macroWidth - i * minSpace,
                spaceRect.getBottom(),
                spaceRect.getRight() - i * minSpace,
                spaceRect.getBottom() + macroHeight);

        if(macroRect.getLeft() >= _searchRect.getLeft() && macroRect.getTop() <= _searchRect.getTop()){
            packEval(tile, macroRect);
        }
    }

    /* place macro from upper left to upper right corner of space tile*/
    for(size_t i = 0; i <= 1; ++i){
        for(size_t j = 0; j <= candCol; ++j){
            macroRect.set4Side(spaceRect.getLeft() + j * pitchH,
                    spaceRect.getTop() - macroHeight - i * minSpace,
                    spaceRect.getLeft() + j * pitchH + macroWidth,
                    spaceRect.getTop() - i * minSpace);

            if(macroRect.getRight() <= _searchRect.getRight() && macroRect.getBottom() >= _searchRect.getBottom()){
                packEval(tile, macroRect);
            }
        }
    }

    /* place macro from upper right to lower right corner of space tile*/
    for(size_t i = 0; i <= 1; ++i){
        for(size_t j = 0; j <= candRow; ++j){
            macroRect.set4Side(spaceRect.getRight() - macroWidth - i * minSpace,
                    spaceRect.getTop() - j * pitchV - macroHeight,
                    spaceRect.getRight() - i * minSpace,
                    spaceRect.getTop() - j * pitchV);

            if(macroRect.getLeft() >= _searchRect.getLeft() && macroRect.getBottom() >= _searchRect.getBottom()){
                packEval(tile, macroRect);
            }
        }
    }
    
    return 0;
}


/*
 *
 *
 *
 */
int TileFindBufferOp::operator() (Tile* tile){
    /* if the type of tile isn't Buffer, nothing to do */
    if(tile->getTileType() != TileType::Buffer)
        return 0;
    
    int powerplanWidth = _legalizer.getPWC();
    Plane* plane = _legalizer.getPlane();
    Plane* bufferPlane = _legalizer.getBufferPlane();
    Rectangle boundingRect = _legalizer.getBoundingRect();
    Rectangle spaceRect, bufferRect;

    spaceRect.tile2Rect(tile);
    spaceRect.clip(_searchRect);

    int candCol = spaceRect.getWidth() / powerplanWidth;
    int candRow = spaceRect.getHeight() / powerplanWidth;
    

    for(size_t i = 0; i <= candRow; ++i){
        for(size_t j = 0; j <= candCol; ++j){
            bufferRect.set4Side(spaceRect.getLeft() + j * powerplanWidth,
                    spaceRect.getBottom() + i * powerplanWidth,
                    spaceRect.getLeft() + (j + 1) * powerplanWidth,
                    spaceRect.getBottom() + (i + 1) * powerplanWidth);
            
            if(bufferRect.overlap(_macroRect)) continue;
            

            if(bufferRect.getRight() <= _searchRect.getRight() && bufferRect.getTop() <= _searchRect.getTop()){
                if(CornerStitch::searchAreaAllBuffer(tile, bufferPlane, &bufferRect) &&
                        CornerStitch::searchAreaAllSpace(NULL, plane, &bufferRect)){
                    double cost = evalCost(bufferRect.getLeft(), bufferRect.getBottom());
                    if(cost < _bestCost){
                        _bestCost = cost;
                        _findResult = true;
                        _bestPoint._x = bufferRect.getLeft();
                        _bestPoint._y = bufferRect.getBottom();
                    }
                }
            }
        }
    }

    

    return 0;
}


/*
 *
 *
 *
 */
void Legalizer::buildBufferPlane(unordered_set<Component*>& freeSpaceVec, Plane* bufferPlane){
    _bufferPlane = bufferPlane;

    Rectangle bufferRect;
    for(unordered_set<Component*>::iterator it = freeSpaceVec.begin(); it != freeSpaceVec.end(); ++it){
        component2Rect((*it), bufferRect);
        Tile* bufferTile = CornerStitch::insertTile(&bufferRect, bufferPlane);
        bufferTile->setTileType(TileType::Buffer);
    }
}


/*
 *
 *
 *
 */
bool Legalizer::findLegalBufferPos(Rectangle macroRect, Rectangle& bufferRect){
    CornerStitch::Point bestPoint;
    bool findBuffer = false;
    TileOp* tileOp = new TileFindBufferOp(*this, 
                                        macroRect,
                                        bestPoint,
                                        findBuffer);
    
    /* search _bufferPlane to find legal buffer area */
    macroRect.scale(_baredc);
    CornerStitch::searchArea(NULL, _bufferPlane, &macroRect, tileOp); 

    delete tileOp;
    
    int powerplanWidth = getPWC();
        
    if(findBuffer){
        bufferRect.set4Side(bestPoint._x,
                bestPoint._y,
                bestPoint._x + powerplanWidth,
                bestPoint._y +powerplanWidth);
    }

    return findBuffer;
}





/*
 *
 * check macroRect rectangle is Legal to place in plane or not
 *
 */
bool Legalizer::checkMacroPos(Rectangle macroRect){
    /* macro rectangle is not all space */
    if(searchAreaAllSpace(NULL, _plane, &macroRect) == false)
        return false;
    /* extand macro rectangle with min space */    
    macroRect.scale(_minSpace);

    /* check min space constraint */
    return searchAreaNoMacro(NULL, _plane, &macroRect);
}


/*
 *
 * check the marco rect extend with BAREDC has at least one buffer area
 *
 */
bool Legalizer::checkMacroBuffer(Rectangle macroRect){
    macroRect.scale(_baredc);
    return CornerStitch::searchAreaHasBuffer(NULL, _plane, &macroRect);
}


/*
 *
 *
 *
 */
bool Legalizer::findLegalMacroPos(Component* macroComp, Rectangle macroRect){
    CornerStitch::Point bestPoint;
    FindType findResult;
    Rectangle bufferRect, bestMacroRect;

    TileOp* tileOp = new TileFindLegalOp(*this,
            macroRect,
            bestMacroRect,
            bufferRect,
            _boundingRect,
            findResult);
    
    CornerStitch::searchArea(NULL, _plane, &_boundingRect, tileOp);
    
    delete tileOp;

    int minSpace = _minSpace;
    
    if(findResult != FindType::Invalid){
        macroComp->_llx = bestMacroRect.getLeft();
        macroComp->_lly = bestMacroRect.getBottom();
        return placeMacro(bestMacroRect);
    }

    if(findResult == FindType::BufferInsert)
        return placeBuffer(bufferRect);

    return false;
}



/*
 *
 * place macro rectangle in plane
 *
 */
bool Legalizer::placeMacro(Rectangle macroRect){

    Tile* macroTile = CornerStitch::insertTile(&macroRect, _plane);

    if(macroTile == NULL)
        return false;

    return true;
}



/*
 *
 * place buffer area in plane
 *
 */
bool Legalizer::placeBuffer(Rectangle bufferRect){
    
    Tile* bufferTile = CornerStitch::insertTile(&bufferRect, _plane);

    if(bufferTile == NULL)
        return false;

    bufferTile->setTileType(TileType::Buffer);
    return true;    
}



/*
 *
 * place the marco with original position or find a new position to place 
 *
 */
bool Legalizer::findAndPlace(Component* macroComp){
    
    Rectangle macroRect, bufferRect;

    /* load the macro rectangle from macro component */
    component2Rect(macroComp, macroRect);

    /* check buffer and macro position */
    bool hasBuffer = _needCheckBuffer ? checkMacroBuffer(macroRect) : true;
    bool posLegal = checkMacroPos(macroRect);

    /* macro position is legal and has at least one buffer area */
    if(hasBuffer && posLegal){
        return placeMacro(macroRect);
    }

    /* macro position is legal but no buffer area */
    if(hasBuffer == false && posLegal){
        /* find a legal position to insert buffer area */
        if(findLegalBufferPos(macroRect, bufferRect)){
            placeBuffer(bufferRect);
            return placeMacro(macroRect);
        }
    }
   
    /* pre-placed macro can't place orignal position */
    if(macroComp->getType() == FIXED)
        return false;

    return findLegalMacroPos(macroComp, macroRect);
}



/*
 * 
 * legalize the macro placement by placing marco one by one.
 *
 */
void Legalizer::legalize(vector<Component*>& nonPlacedVec){

    Rectangle macroRect;
    int minSpace = _minSpace;
    int bufferSpace = _baredc;

    /* place pre-placed macros into corner stitching tile plane */
    for(size_t i = 0; i < _compVec.size(); ++i){
        // place each pre-placed macro 
        if(_compVec[i]->getType() == FIXED){
            findAndPlace(_compVec[i]); 
        }
    }
   
    /* find solve macros to put into legalMacroVec */
    vector<Component*> legalMacroVec;
    for(size_t i = 0; i < _compVec.size(); ++i){
        bool hasFound = false;

        /* check current component in the nonPlacedVec  */
        for(size_t j = 0; j < nonPlacedVec.size(); ++j){
            if(_compVec[i]->getName() == nonPlacedVec[j]->getName()){
                hasFound = true;
                break;
            }
        }

        /* not found in nonPlacedVec meaning it legal */
        if(hasFound == false)
            legalMacroVec.push_back(_compVec[i]);
    }

    for(size_t i = 0; i < legalMacroVec.size(); ++i)
        if(legalMacroVec[i]->getType() == PLACED)
            findAndPlace(legalMacroVec[i]);

    /**/
    vector<Component*> macroOrderVec = nonPlacedVec;
    std::sort(macroOrderVec.begin(), macroOrderVec.end(), macroCompare);
    

    /* place unsolve macros into corner stitching tile plane */
    cout << "unsolve macro number: " << macroOrderVec.size() << endl;
    for(size_t i = 0; i < macroOrderVec.size(); ++i){
        if(macroOrderVec[i]->getType() == PLACED){
            findAndPlace(macroOrderVec[i]);
        }
    }
}

/*
 * */
double TileFindLegalOp::evalCost(int x, int y){
    return abs(_macroRect.getLeft() - x) + abs(_macroRect.getBottom() - y);
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
    else if(tile->getTileType() == TileType::Buffer)
        _outgraph << " fc rgb '#00ff00'";
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
    
    TileOp* tileOp = new TileDrawOp(outgraph, _boundingRect, getMCSBMC());
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







void Legalizer::bufferPlot(){
    fstream outgraph("legalizer.gp", ios::out);

    outgraph << "reset\n";
    outgraph << "set tics\n";
    outgraph << "unset key\n";
    outgraph << "set title \"The result of Corner Stitching\"\n";
    
    TileOp* tileOp = new TileDrawOp(outgraph, _boundingRect, getMCSBMC());
    CornerStitch::searchArea(NULL, _bufferPlane, &_boundingRect, tileOp);
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




