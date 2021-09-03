#include "tile.h"

using namespace std;
using namespace CornerStitch;

namespace CornerStitch{


    static Tile* splitTileX(Tile* tile, int _cx);
    static Tile* splitTileY(Tile* tile, int _cy);
    static void  mergeTileX(Tile* tile1, Tile* tile2, Plane* plane);
    static void  mergeTileY(Tile* tile1, Tile* tile2, Plane* plane);


    /* rectangle constructor with 4 boundaries */
    Rectangle::Rectangle(int left, int bottom, int right, int top){
        set4Side(left, bottom, right, top);
    }

    /* set 4 boundaries of the rectangle */
    void Rectangle::set4Side(int left, int bottom, int right, int top){
        setLeft(left);
        setBottom(bottom);
        setRight(right);
        setTop(top);
    }

    /* clip the rectangle with given region */
    void Rectangle::clip(Rectangle& clipRegion){
        _ll._x = MAX(_ll._x, clipRegion.getLeft());
        _ll._y = MAX(_ll._y, clipRegion.getBottom());
        _ur._x = MIN(_ur._x, clipRegion.getRight());
        _ur._y = MIN(_ur._y, clipRegion.getTop());
    }

    /* covert tile to rectangle */
    void Rectangle::tile2Rect(Tile* tile){
        _ll._x = tile->getLeft();
        _ll._y = tile->getBottom();
        _ur._x = tile->getRight();
        _ur._y = tile->getTop();
    }

    /* enlarge or shrink the rectangle area */
    void Rectangle::scale(int range){
        _ll._x -= range;
        _ll._y -= range;
        _ur._x += range;
        _ur._y += range;
    }
    
    void Rectangle::scaleUR(int range){
        _ur._x += range;
        _ur._y += range;
    }

    /*
     * */
    Plane::Plane(Rectangle& validRegion){
        /* plane class only has one infinityTile and
         * should use other design pattern for future.
         * */
        static Tile* infinityTile = NULL;

        /* first initialize infinityTile */
        if(infinityTile == NULL){
            infinityTile = new Tile(TileType::Invalid);
            infinityTile->setLeft(INF + 1);
            infinityTile->setBottom(INF + 1);
        }
        
        /* create world tile (NINF + 1, NINF + 1) -> (INF, INF) */
        Rectangle worldRegion(NINF + 1, NINF + 1, INF, INF);
        Tile* worldTile = new Tile(TileType::Space);
        worldTile->setLeft(NINF + 1);
        worldTile->setBottom(NINF + 1);

        /* create 4 boundary tiles */
        _left = new Tile(TileType::Invalid);
        _right = new Tile(TileType::Invalid);
        _bottom = new Tile(TileType::Invalid);
        _top = new Tile(TileType::Invalid);

        /* link 4 pointers to 4 boundary tiles, separately */
        worldTile->_bl = _left;
        worldTile->_tr = _right;
        worldTile->_lb = _bottom;
        worldTile->_rt = _top;
            

        // set left boundary
        _left->setLeft(NINF);
        _left->setBottom(NINF);
        _left->_bl = NULL;
        _left->_tr = worldTile;
        _left->_lb = _bottom;
        _left->_rt = _top;

        // set right boundary
        _right->setLeft(INF);
        _right->setBottom(NINF);
        _right->_bl = worldTile;
        _right->_tr = infinityTile;
        _right->_lb = _bottom;
        _right->_rt = _top;

        // set bottom boundary
        _bottom->setLeft(NINF);
        _bottom->setBottom(NINF);
        _bottom->_bl = _left;
        _bottom->_tr = _right;
        _bottom->_lb = NULL;
        _bottom->_rt = worldTile;

        // set top boundary
        _top->setLeft(NINF);
        _top->setBottom(INF);
        _top->_bl = _left;
        _top->_tr = _right;
        _top->_lb = worldTile;
        _top->_rt = infinityTile;

        // set hint tile
        _hintTile = worldTile;

        /* create validTile with valid region */
        Tile* validTile = insertTile(&validRegion, this);

        TileOp* tileOp = new TileOp();
        searchArea(NULL, this, &worldRegion, tileOp);
        delete tileOp;

        validTile->setTileType(TileType::Space);
    }

    /*
     * */
    Plane::~Plane(){
        delete _left;
        delete _right;
        delete _bottom;
        delete _top;
    }

    /*
     * */
    Tile* searchPoint(Tile* hintTile, Plane* plane, Point* point){
        Tile* tp = (hintTile != NULL) ? hintTile : plane->getHintTile();
        // vertical range
        if(point->_y < tp->getBottom())
            for(tp = tp->_lb; point->_y < tp->getBottom(); tp = tp->_lb);
        else
            for(; point->_y >= tp->getTop(); tp = tp->_rt);

        // horizontal range
        if(point->_x < tp->getLeft())
            do{
                for(tp = tp->_bl; point->_x < tp->getLeft(); tp = tp->_bl);
                if(point->_y < tp->getTop())
                    break;
                for(tp = tp->_rt; point->_y >= tp->getTop(); tp = tp->_rt);
            }while(point->_x < tp->getLeft());
        else
            while(point->_x >= tp->getRight()){
                for(tp = tp->_tr; point->_x >= tp->getRight(); tp = tp->_tr);
                if(point->_y >= tp->getBottom())
                    break;
                for(tp = tp->_lb; point->_y < tp->getBottom(); tp = tp->_lb);
            }
        
        // update hint tile
        plane->setHintTile(tp);
        
        return tp;
    }


    /*
     * */
    int searchArea(Tile* hintTile, Plane* plane, Rectangle* rect, TileOp* tileOp){
        Point here;
        here._x = rect->getLeft();
        here._y = rect->getTop() - 1;
        Tile* enumTile = (hintTile != NULL) ? hintTile : plane->getHintTile();
        enumTile = searchPoint(enumTile, plane, &here);

        Tile* tp;
        while(here._y >= rect->getBottom()){
            here._y = enumTile->getBottom() - 1;
            tp = enumTile;
            tp = searchPoint(tp, plane, &here);

            int enumRight = enumTile->getRight();
            int enumBottom = enumTile->getBottom();
            Tile* enumTR = enumTile->_tr;

           if((*tileOp)(enumTile))
                return 1;
            
            if(enumRight < rect->getRight())
                if(searchAreaEnum(enumTR, enumBottom, rect, tileOp))
                    return 1;

            enumTile = tp;
        }

        return 0;
    }

    /*
     * */
    int searchAreaEnum(Tile* enumRT, int enumBottom, Rectangle* rect, TileOp* tileOp){
        int atBottom = (enumBottom <= rect->getBottom());
        int tpRight, tpNextTop, tpBottom, searchBottom = enumBottom;

        if(searchBottom < rect->getBottom())
            searchBottom = rect->getBottom();
        
        Tile* tp;
        Tile* tpLB;
        Tile* tpTR;
        for(tp = enumRT, tpNextTop = tp->getTop(); tpNextTop > searchBottom; tp = tpLB){
            tpLB = tp->_lb;
            tpNextTop = tpLB->getTop();

            if(tp->getBottom() < rect->getTop() && (atBottom || tp->getBottom() >= enumBottom)){
                tpRight = tp->getRight();
                tpBottom = tp->getBottom();
                tpTR = tp->_tr;

               if((*tileOp)(tp))
                    return 1;

                if(tpRight < rect->getRight())
                    if(CornerStitch::searchAreaEnum(tpTR, tpBottom, rect, tileOp))
                        return 1;
            
            }
        }

        return 0;
    }


    bool canMergeVertical(Tile* tile1, Tile* tile2){
        if(tile1->_tileType != TileType::Space) return false;
        if(tile2->_tileType != TileType::Space) return false;
        if(tile1->getLeft() != tile2->getLeft()) return false;
        if(tile1->getRight() != tile2->getRight()) return false;

        return true;
    }

    /*
     * */
    Tile* insertTile(Rectangle* rect, Plane* plane){
        Point ll = rect->getLL();
        Tile* tp = searchPoint(NULL, plane, &ll);


        if(tp->getBottom() < rect->getBottom())
            tp = splitTileY(tp, rect->getBottom()); 


        Tile* target = NULL;
        while(tp->getTop() <= rect->getTop()){
            if(tp->getLeft() < rect->getLeft()){
                Tile* oldTile = tp;
                tp = splitTileX(tp, rect->getLeft());
                Tile* btp = oldTile->_lb;
                if(canMergeVertical(oldTile, btp))
                    mergeTileY(oldTile, btp, plane);
            }
            if(tp->getRight() > rect->getRight()){
                Tile *newTile = splitTileX(tp, rect->getRight());
                Tile* btp = newTile->_lb;
                if(canMergeVertical(newTile, btp))
                    mergeTileY(newTile, btp, plane);
            }
            if(target != NULL)
                mergeTileY(tp, target, plane);
            target = tp;

            tp = tp->_rt;
        }

        if(tp->getBottom() < rect->getTop() && tp->getTop() > rect->getTop()){
            splitTileY(tp, rect->getTop());

            if(tp->getLeft() < rect->getLeft()){
                Tile* oldTile = tp;
                tp = splitTileX(tp, rect->getLeft());

                Tile* btp = oldTile->_lb;
                if(canMergeVertical(oldTile, btp))
                    mergeTileY(oldTile, btp, plane);
            }
            if(tp->getRight() > rect->getRight()){
                Tile *newTile = splitTileX(tp, rect->getRight());

                Tile* btp = newTile->_lb;
                if(canMergeVertical(newTile, btp))
                    mergeTileY(newTile, btp, plane);
            }
            if(target != NULL)
                mergeTileY(tp, target, plane);
            target = tp;
        }

        target->_tileType = TileType::Macro;
        return target;
    }


    /*
     *
     * */
    bool searchAreaAllSpace(Tile* hintTile, Plane* plane, Rectangle* rect){
        bool isAllSpace = true;
        TileOp* tileOp = new TileIsSpaceOp(isAllSpace);

        searchArea(hintTile, plane, rect, tileOp);

        delete tileOp;

        return isAllSpace;
    }

    /*
     *
     *
     *
     */
    bool searchAreaNoMacro(Tile* hintTile, Plane* plane, Rectangle* rect){
        bool noneMacro = true;
        TileOp* tileOp = new TileNotMacroOp(noneMacro);

        searchArea(hintTile, plane, rect, tileOp);

        delete tileOp;

        return noneMacro;
    }



    /*
     *
     *
     *
     */
    void fillArea(Tile* hintTile, Plane* plane, Rectangle* rect){
        vector<Rectangle> fillRegionVec;
        TileOp* tileOp = new TileFillOp(fillRegionVec, *rect);

        searchArea(hintTile, plane, rect, tileOp);

        delete tileOp;

        for(size_t i = 0; i < fillRegionVec.size(); ++i)
            insertTile(&fillRegionVec[i], plane);
    }



    /*
     * */
    Tile* splitTileX(Tile* tile, int _cx){
        Tile* rightTile = new Tile();
        
        rightTile->setLeft(_cx);
        rightTile->setBottom(tile->getBottom());
        rightTile->_bl = tile;
        rightTile->_tr = tile->_tr;
        rightTile->_rt = tile->_rt;

        Tile* tp;
        // corner stitches along the right edge
        for(tp = tile->_tr; tp->_bl == tile; tp = tp->_lb)
            tp->_bl = rightTile;
        tile->_tr = rightTile;

        // corner stitches along the top edge
        for(tp = tile->_rt; tp->getLeft() >= _cx; tp = tp->_bl)
            tp->_lb = rightTile;
        tile->_rt = tp;

        // corner stitches along the bottom edge
        for(tp = tile->_lb; tp->getRight() <= _cx; tp = tp->_tr);
        rightTile->_lb = tp;
        for(; tp->_rt == tile; tp = tp->_tr)
            tp->_rt = rightTile;

        return rightTile;
    }

    /*
     * */
    Tile* splitTileY(Tile* tile, int _cy){
        Tile* topTile = new Tile();
        
        topTile->setLeft(tile->getLeft());
        topTile->setBottom(_cy);
        topTile->_lb = tile;
        topTile->_rt = tile->_rt;
        topTile->_tr = tile->_tr;

        Tile* tp;
        // corner stitches along the top edge
        for(tp = tile->_rt; tp->_lb == tile; tp = tp->_bl)
            tp->_lb = topTile;
        tile->_rt = topTile;

        // corner stitches along the right edge
        for(tp = tile->_tr; tp->getBottom() >= _cy; tp = tp->_lb)
            tp->_bl = topTile;
        tile->_tr = tp;

        // corner stitches along the left edge
        for(tp = tile->_bl; tp->getTop() <= _cy; tp = tp->_rt);
        topTile->_bl = tp;
        for(; tp->_tr == tile; tp = tp->_rt)
            tp->_tr = topTile;

        return topTile;
    }

    /*
     * */
    void mergeTileX(Tile* tile1, Tile* tile2, Plane *plane){
        Tile* tp;
        // corner stitches along the top of edge
        for(tp = tile2->_rt; tp->_lb == tile2; tp = tp->_bl)
            tp->_lb = tile1;
        
        // corner stitches along the bottom of edge
        for(tp = tile2->_lb; tp->_rt == tile2; tp = tp->_tr)
            tp->_rt = tile1;

        // tile1 is left than tile2
        if(tile1->getLeft() < tile2->getLeft()){
            // corner stitches along the right of tile2
            for(tp = tile2->_tr; tp->_bl == tile2; tp = tp->_lb)
                tp->_bl = tile1;
            tile1->_rt = tile2->_rt;
            tile1->_tr = tile2->_tr;
        }
        else{
            // corner stitches along the left of tile2
            for(tp = tile2->_bl; tp->_tr == tile2; tp = tp->_rt)
                tp->_tr = tile1;
            tile1->_bl = tile2->_bl;
            tile1->_lb = tile2->_lb;
            tile1->setLeft(tile2->getLeft());
        }

        // update hint tile if hint tile is tile2
        if(plane->getHintTile() == tile2)
            plane->setHintTile(tile1);

        delete tile2;
    }

    /*
     * */
    void mergeTileY(Tile* tile1, Tile* tile2, Plane* plane){
        Tile* tp;
        // corner stitches along the right of edge
        for(tp = tile2->_tr; tp->_bl == tile2; tp = tp->_lb)
            tp->_bl = tile1;

        // corner stitches along the left of edge
        for(tp = tile2->_bl; tp->_tr == tile2; tp = tp->_rt)
            tp->_tr = tile1;

        // tile1 is lower than tile2
        if(tile1->getBottom() < tile2->getBottom()){
            // corner stitches along the top of tile2
            for(tp = tile2->_rt; tp->_lb == tile2; tp = tp->_bl)
                tp->_lb = tile1;
            tile1->_rt = tile2->_rt;
            tile1->_tr = tile2->_tr;
        }
        else{
            // corner stitches along the bottom of tile2
            for(tp = tile2->_lb; tp->_rt == tile2; tp = tp->_tr)
                tp->_rt = tile1;
            tile1->_lb = tile2->_lb;
            tile1->_bl = tile2->_bl;
            tile1->setBottom(tile2->getBottom());
        }

        // update hint tile if hint tile is tile2
        if(plane->getHintTile() == tile2)
            plane->setHintTile(tile1);
        
        delete tile2;
    }
}


