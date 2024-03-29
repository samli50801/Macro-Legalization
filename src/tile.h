#ifndef TILE_H
#define TILE_H

#include<iostream>
#include<vector>

using namespace std;

#define INF ((1 << 30) - 4)
#define NINF (-INF)
#define MAX(a, b) ((a >= b) ? a : b)
#define MIN(a, b) ((a <= b) ? a : b)

namespace CornerStitch{
    
    class Tile;

    /* class definition for Point */
    class Point{
        public:
            void print() { cout << "(" << _x << ", " << _y << ")"; }

            int _x;
            int _y;
    };

    /* class definition for Rectangle */
    class Rectangle{
        public:
            Rectangle(){}
            Rectangle(int left, int bottom, int right, int top);

            /* set 4 sides of the rectangle */
            void setLeft(int left) { _ll._x = left; }
            void setRight(int right) { _ur._x = right; }
            void setBottom(int bottom) { _ll._y = bottom; }
            void setTop(int top) { _ur._y = top; }
            void set4Side(int left, int bottom, int right, int top);

            /* get 4 sides of the rectangle */
            int getLeft() { return _ll._x; }
            int getRight() { return _ur._x; }
            int getBottom() { return _ll._y; }
            int getTop() { return _ur._y; }

            /* get width and height of the rectangle */
            int getWidth() { return getRight() - getLeft(); }
            int getHeight() { return getTop() - getBottom(); }
           
            /* get lower-left and upper-right point */
            Point getLL() { return _ll; }
            Point getUR() { return _ur; }
            
            /* clip the rectangle with given region */
            void clip(Rectangle& clipRegion);
            
            /* covert tile to rectangle */
            void tile2Rect(Tile* tile);

            /* enlarge or shrink the rectangle area */
            void scale(int range);
            void scaleUR(int range);
            
            void translate(int dx, int dy){
                _ll._x += dx;
                _ur._x += dx;
                _ll._y += dy;
                _ur._y += dy;
            }

            bool overlap(Rectangle& rect){
                int x = MAX(getLeft() - rect.getRight(), rect.getLeft() - getRight());
                int y = MAX(getBottom() - rect.getTop(), rect.getBottom() - getTop());

                if(MAX(x, y) < 0)
                    return true;
                else
                    return false;
            }

            void print() { _ll.print(); cout << " - > "; _ur.print(); cout << endl; }
        
            Point   _ll;    // lower-left corner of rectangle
            Point   _ur;    // upper-right corner of rectangle
    };

    enum class TileType { Invalid, Space, Macro, Boundary, Buffer };

    /* 
     *
     * a tile is the basic unit used for representing both space and solid area in a plane.
     * only record lower-left coordinates of tile.
     *
     */
    class Tile{
        public:
            Tile() : _tileType(TileType::Space) {}
            Tile(TileType tileType) : _tileType(tileType) {}

            int getLeft() { return _ll._x; }
            int getRight() { return _tr->getLeft(); }
            int getBottom() { return _ll._y; }
            int getTop() { return _rt->getBottom(); }
            TileType getTileType() { return _tileType; }


            void setLeft(int llx) { _ll._x = llx; }
            void setBottom(int lly) { _ll._y = lly; }
            void setTileType(TileType tileType) { _tileType = tileType; }
            
            //friend ostream& operator<<(ostream& os, Tile &tile);

            Tile*       _lb;         // leftmost bottom corner stitch
            Tile*       _bl;         // bottom left corner stitch
            Tile*       _tr;         // topmost right corner stitch
            Tile*       _rt;         // rightmost top corner stitch
            Point       _ll;         // lower-left coordinate of tile
            TileType    _tileType;   // type of tile
    };


    /*
     *
     * a plane of tiles consists of the four special tiles
     *
     */
    class Plane{
        public:
            Plane(Rectangle& validRegion);
            ~Plane();
            
            /* get hint tile of this plane */
            Tile* getHintTile() { return _hintTile; }
            
            /* set hint tile of this plane */
            void setHintTile(Tile* hintTile) { _hintTile = hintTile; }

        private:
            Tile*   _left;      // left boundary
            Tile*   _right;     // right boundary
            Tile*   _bottom;    // bottom boundary
            Tile*   _top;       // top boundary
            Tile*   _hintTile;  // begin searching tile
    };


    /*
     *
     * base class of tile operation for searchArea and searchAreaEnum
     * functions in CornerStitching implementation with functor. All
     * inheritance class need override operator().
     *
     * */
    class TileOp{
        public:
            virtual ~TileOp() {}
            virtual int operator() (Tile* tile){
                tile->setTileType(TileType::Invalid);
                return 0;
            }

    };


    /*
     *
     * tile is space operation: for each tile being traversed, check whether 
     * the tile type is TileType::Space or not. if one of tile type
     * isn't Space, then _isAllSpace is false.
     *
     */
    class TileIsAllOp : public TileOp{
        public:
            TileIsAllOp(bool &isAll, vector<TileType> tileTypeVec) : 
                _isAll(isAll), _tileTypeVec(tileTypeVec) {}
            virtual ~TileIsAllOp() {}

            virtual int operator() (Tile* tile){
                bool isOneOfType = false;
                for(size_t i = 0; i < _tileTypeVec.size(); ++i){
                    if(tile->getTileType() == _tileTypeVec[i]){
                        isOneOfType = true;
                        break;
                    }
                }

                if(isOneOfType)
                    return 0;

                _isAll = false;
                return 1;
            }

        private:
            bool&                  _isAll;
            vector<TileType>       _tileTypeVec;
    };

        
    /*
     *
     * tile not macro operation: for each tile being traversed, check whether 
     * the tile type is TileType::Macro or not. if one of tile type
     * is Macro, then _noneMacro is false.
     *
     */
    class TileIsAllNotOp : public TileOp{
        public:
            TileIsAllNotOp(bool &isAllNot, vector<TileType> tileTypeVec) : 
                _isAllNot(isAllNot), _tileTypeVec(tileTypeVec) {}
            virtual ~TileIsAllNotOp() {}

            virtual int operator() (Tile* tile){
                bool isOneOfType = false;
                for(size_t i = 0; i < _tileTypeVec.size(); ++i){
                    if(tile->getTileType() == _tileTypeVec[i]){
                        isOneOfType = true;
                        break;
                    }
                }

                if(isOneOfType){
                    _isAllNot = false;
                    return 1;
                }

                return 0;
            }

        private:
            bool&                  _isAllNot;
            vector<TileType>       _tileTypeVec;
    };


    /*
     *
     * tile is buffer operation: for each tile being traversed, check whether 
     * the tile type is TileType::Buffer or not. if one of tile type
     * is Buffer, then _hasBuffer is true.
     *
     */
    class TileIsBufferOp : public TileOp{
        public:
            TileIsBufferOp(bool &hasBuffer) : _hasBuffer(hasBuffer) {}
            virtual ~TileIsBufferOp() {}

            virtual int operator() (Tile* tile){
                if(tile->_tileType == TileType::Buffer){
                    _hasBuffer = true;
                    return 1;
                }
                return 0;
            }

        private:
            bool&   _hasBuffer;
    };

    
    /*
     *
     *
     *
     */
    class TileAllBufferOp : public TileOp{
        public:
            TileAllBufferOp(bool &isAllBuffer) : _isAllBuffer(isAllBuffer) {}
            virtual ~TileAllBufferOp() {}

            virtual int operator() (Tile* tile){
                if(tile->getTileType() != TileType::Buffer){
                    _isAllBuffer = false;
                    return 1;
                }
                return 0;
            }

        private:
            bool&   _isAllBuffer;
    };



    /*
     *
     * tile fill operation:
     *
     */
    class TileFillOp: public TileOp{
        public:
            TileFillOp(vector<Rectangle>& rectVec, Rectangle& fillRegion) 
                : _rectVec(rectVec), _fillRegion(fillRegion) {}
            virtual ~TileFillOp() {}

            virtual int operator() (Tile* tile){
                if(tile->_tileType == TileType::Space){
                    Rectangle rect;
                    rect.tile2Rect(tile);
                    rect.clip(_fillRegion);
                    _rectVec.push_back(rect);
                }

                return 0;
            }
        private:
            vector<Rectangle>&  _rectVec;
            Rectangle&          _fillRegion;
    };

    
    /*
     *
     * 
     *
     */
    class TileGetOp: public TileOp{
        public:
            TileGetOp(vector<Tile*>& tileVec) : _tileVec(tileVec){}
            virtual ~TileGetOp() {};

            virtual int operator() (Tile* tile){
                _tileVec.push_back(tile);
                return 0;
            }

        private:
            vector<Tile*>& _tileVec;
    };


    /*
     *
     * CornerStitching defines a series of operations on Plane and Tile.
     *
     */

    Tile* searchPoint(Tile* hintTile, Plane* plane, Point* point);
    int searchArea(Tile* hintTile, Plane* plane, Rectangle* rect, TileOp* tileOp);
    int searchAreaEnum(Tile* enumRT, int enumBottom, Rectangle* rect, TileOp* tileOp);
    Tile* insertTile(Rectangle* rect, Plane* plane);

    bool searchAreaAllSpace(Tile* hintTile, Plane* plane, Rectangle* rect);
    bool searchAreaNoMacro(Tile* hintTile, Plane* plane, Rectangle* rect);
    bool searchAreaHasBuffer(Tile* hintTile, Plane* plane, Rectangle* rect);
    bool searchAreaAllBuffer(Tile* hintTile, Plane* plane, Rectangle* rect);
    void fillMinspace(Tile* hintTile, Plane* plane, Rectangle* rect);
    bool searchBufferArea(Tile* hintTile, Plane* plane, Rectangle* rect);

};

#endif
