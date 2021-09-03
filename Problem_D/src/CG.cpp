#include "CG.h"
#include "../lp_solve/lp_lib.h"
#include <numeric>

using namespace cg;

bool CG::startBufferAreaOpt = false;
bool CG::startBufferAreaLeg = false;
vector<Component*> CG::lastDeletedPoint;

bool AreaComp(const Point* a, const Point* b) {
    return a->_comp->getArea() < b->_comp->getArea();
}

Point::Point(Component* comp)
{
    /* pseudo source and sink */
    if (comp == NULL) {
        Component *pseudoComp = new Component();
        pseudoComp->type = FIXED;
        _comp = pseudoComp;
    }else 
    /* macros */
    {  
        _comp = comp;
        _lm = -1;
        _rm = INT_MAX;
        _dm = -1;
        _um = INT_MAX;
    }  
}

void CG::addBoundaryPoint()
{
    for (size_t i = 0; i < _boundaryComp.size(); ++i) {
        Point *newPoint = new Point(_boundaryComp[i]);
        newPoint->_no = _pointVec.size();
        _pointVec.push_back(newPoint);
    }
}

void CG::initialize()
{
    /* _pointVec initialization */
    _pointVec.clear();
    _pointVec.reserve(_compVec.size() + _boundaryComp.size() + 4);

    /* component to point */
    for (size_t i = 0; i < _compVec.size(); ++i) {
        Point *newPoint = new Point(_compVec[i]);
        newPoint->_no = i;
        _pointVec.push_back(newPoint);
    }

    /* boundary tile to point */
    addBoundaryPoint();

    /* pseudopoint initialization */
    _hsource = new Point(NULL);
    _hsource->_no = _pointVec.size();
    _hsource->_comp->name = "_hsource";
    _hsource->_comp->setArg(_designLeft, _designDown, 0, _designUp - _designDown);

    _hsink = new Point(NULL);
    _hsink->_no = _pointVec.size() + 1;
    _hsink->_comp->name = "_hsink";
    _hsink->_comp->setArg(_designRight, _designDown, 0, _designUp - _designDown);

    _vsource = new Point(NULL);
    _vsource->_no = _pointVec.size() + 2;
    _vsource->_comp->name = "_vsource";
    _vsource->_comp->setArg(_designLeft, _designDown, _designRight - _designLeft, 0);

    _vsink = new Point(NULL);
    _vsink->_no = _pointVec.size() + 3;
    _vsink->_comp->name = "_vsink";
    _vsink->_comp->setArg(_designLeft, _designUp, _designRight - _designLeft, 0);
}

int CG::getRelation(Point* i, Point* j) // j is on the i's xxx
{
    int right = j->_comp->get_ll_x() - i->_comp->get_ur_x();  //then j is Totally to right Of i +
    int left = i->_comp->get_ll_x() - j->_comp->get_ur_x();   //then j is Totally to left Of i +
    int up = j->_comp->get_ll_y() - i->_comp->get_ur_y();     //then j is Totally above i + 
    int down = i->_comp->get_ll_y() - j->_comp->get_ur_y();   //then j is Totally below i +

    /* upper-right */
    if (right >= 0 && up >= 0) {
        if (right >= up) 
            return RIGHT;
        else 
            return UP;
    }
    /* lower-right */
    else if (right >= 0 && down >= 0) {
		if (right >= down) 
            return RIGHT;
		else
		    return DOWN;
	}
    /* upper-left */
    else if (left >= 0 && up >= 0) {
        if (left >= up) 
            return LEFT;
        else
            return UP;
    }
    /* lower-left */
    else if (left >= 0 && down >= 0) {
        if (left >= down) 
            return LEFT;
        else 
            return DOWN;
    }
    /* right */
    else if (right >= 0) return RIGHT;
    /* up */
	else if (up >= 0) return UP;
    /* down */
	else if (down >= 0) return DOWN;
    /* left */
	else if (left >= 0) return LEFT;
    /* overlap */
    else if (!(right>=0 || left>=0 || up>=0 || down>=0)) {

        Component* a = i->_comp;
        Component* b = j->_comp;
        double xOverlap = (a->getWidth() + b->getWidth()) / 2.0 - fabs(a->get_mid_x() - b->get_mid_x());
        double yOverlap = (a->getHeight() + b->getHeight()) / 2.0 - fabs(a->get_mid_y() - b->get_mid_y());
        if (xOverlap <= yOverlap) {
            double right = fabs(a->get_ll_x() - b->get_ur_x());
		    double left = fabs(a->get_ur_x() - b->get_ll_x());
            // i is right of j
		    if (right < left) 
			    return LEFT;
		    else 
                return RIGHT;
        }else {
            double up = fabs(a->get_ll_y() - b->get_ur_y());
		    double down = fabs(a->get_ur_y() - b->get_ll_y());
		    // i is above j
		    if (up < down) 
                return DOWN; 
		    else
                return UP;
        }
    }
}

bool CG::optimizeBufferArea(Point* a, Point* b, bool relation)
{
    if (a->_comp->getType() == FIXED || b->_comp->getType() == FIXED)
        return false;
    /* one of them has been deleted in last iteration*/
    vector<Component*>::iterator find_a = std::find(deletedPoint.begin(), deletedPoint.end(), a->_comp);
    vector<Component*>::iterator find_b = std::find(deletedPoint.begin(), deletedPoint.end(), b->_comp);
    if (find_a != deletedPoint.end() || find_b != deletedPoint.end()) 
        return false;

    double threashold = 0.6;    //best: 0.6
    
    /* a is left, b is right*/
    if (relation == H) {
        double overlapLength = std::max(0, min(a->_comp->get_ur_y(), b->_comp->get_ur_y()) - max(a->_comp->get_ll_y(), b->_comp->get_ll_y()));
        // won't have dead space 
        if (overlapLength == 0)
            return false;
        double distance = (double)b->_comp->get_ll_x() - (double)a->_comp->get_ur_x();
        // probably won't have dead space
        if (distance >= _pwc) 
            return false;

        double currentDeadSpace = (double)overlapLength * (double)distance;

        double leftDeadSpace = (double)_minSpace * (double)overlapLength;
        double leftMovement = abs(distance - _minSpace);

        double rightDeadSpace = 0.0;
        double rightMovement = abs(_pwc - distance);

        double leftCost = _parser._wa * leftMovement + _parser._wb * (sqrt(leftDeadSpace) - sqrt(currentDeadSpace));
        double rightCost = _parser._wa * rightMovement + _parser._wb * (sqrt(rightDeadSpace) - sqrt(currentDeadSpace));
        
        /*if (_parser._wa * rightMovement < _parser._wb * sqrt(currentDeadSpace))
            return true;
        else 
            return false;*/

        /*if (leftCost < 0.0 && leftCost < rightCost) {
            return false;
        } else if (rightCost < 0.0 && rightCost < leftCost) {
            return true;
        }*/
        if (distance > (double)_pwc * threashold)
            return true;
        return false;
    }   
    else if (relation == V) {
        double overlapLength = std::max(0, min(a->_comp->get_ur_x(), b->_comp->get_ur_x()) - max(a->_comp->get_ll_x(), b->_comp->get_ll_x()));
        // won't have dead space 
        if (overlapLength == 0)
            return false;
        double distance = (double)b->_comp->get_ll_y() - (double)a->_comp->get_ur_y();
        // probably won't have dead space
        if (distance >= (double)_pwc) 
            return false;

        double currentDeadSpace = (double)overlapLength * (double)distance;

        double leftDeadSpace = (double)_minSpace * (double)overlapLength;
        double leftMovement = abs(distance - _minSpace);

        double rightDeadSpace = 0.0;
        double rightMovement = abs(_pwc - distance);

        double leftCost = _parser._wa * leftMovement + _parser._wb * (sqrt(leftDeadSpace) - sqrt(currentDeadSpace));
        double rightCost = _parser._wa * rightMovement + _parser._wb * (sqrt(rightDeadSpace) - sqrt(currentDeadSpace));
        
        /*if (_parser._wa * rightMovement < _parser._wb * sqrt(currentDeadSpace))
            return true;
        else 
            return false;*/
        /*if (leftCost < 0.0 && leftCost < rightCost) {
            return false;
        } else if (rightCost < 0.0 && rightCost < leftCost) {
            return true;
        }*/
        if (distance > (double)_pwc * threashold) 
            return true;
        return false;
    }   

}

void CG::buildGraph()
{
    /* build the TCG */
    /* total n*(n-1)/2 comparisons */
    for (size_t i = 0; i < _pointVec.size() - 1; ++i) {
        for (size_t j = i + 1; j < _pointVec.size(); ++j) {
            /* j is on i which direction */
            int relation = getRelation(_pointVec[i], _pointVec[j]);

            /* edge weight */
            int piWidth = _pointVec[i]->_comp->width + _minSpace;
            int piHeight = _pointVec[i]->_comp->height + _minSpace;
            int pjWidth = _pointVec[j]->_comp->width + _minSpace;
            int pjHeight = _pointVec[j]->_comp->height + _minSpace;

            /* no need to leave minSpace between macro and boundary */
            if (_pointVec[i]->_comp->name == "boundaryComp" || _pointVec[j]->_comp->name == "boundaryComp"){
                piWidth -= _minSpace;
                piHeight -= _minSpace;
                pjWidth -= _minSpace;
                pjHeight -= _minSpace;
            }

            /* build edge */
            switch (relation)
            {
            case LEFT:
                if (startBufferAreaOpt && optimizeBufferArea(_pointVec[j], _pointVec[i], H))
                    pjWidth = _pointVec[j]->_comp->width + _pwc;
                _pointVec[j]->_right.push_back(make_pair(i, pjWidth));
                _pointVec[i]->_left.push_back(make_pair(j, pjWidth));
                break;
            case RIGHT:
                if (startBufferAreaOpt && optimizeBufferArea(_pointVec[i], _pointVec[j], H))
                    piWidth = _pointVec[i]->_comp->width + _pwc;
                _pointVec[i]->_right.push_back(make_pair(j, piWidth));
                _pointVec[j]->_left.push_back(make_pair(i, piWidth));
                break;
            case DOWN:
                if (startBufferAreaOpt && optimizeBufferArea(_pointVec[j], _pointVec[i], V))
                    pjHeight = _pointVec[j]->_comp->height + _pwc;
                _pointVec[j]->_up.push_back(make_pair(i, pjHeight));
                _pointVec[i]->_down.push_back(make_pair(j, pjHeight));
                break;
            case UP:
                if (startBufferAreaOpt && optimizeBufferArea(_pointVec[i], _pointVec[j], V))
                    piHeight = _pointVec[i]->_comp->height + _pwc;
                _pointVec[i]->_up.push_back(make_pair(j, piHeight));
                _pointVec[j]->_down.push_back(make_pair(i, piHeight));
                break;
                
            default:
                break;
            }
        }
    }

    /* connect the node who has zero InDegree(OutDegree) to the source(sink) */
    for (size_t i = 0; i < _pointVec.size(); ++i) {
        if (_pointVec[i]->_left.empty()) {
            _hsource->_right.push_back(make_pair(i, _hsource->_comp->width));
            _pointVec[i]->_left.push_back(make_pair(_hsource->_no, _hsource->_comp->width));
        }
        if (_pointVec[i]->_right.empty()) {
            _hsink->_left.push_back(make_pair(i, _pointVec[i]->_comp->getWidth()));
            _pointVec[i]->_right.push_back(make_pair(_hsink->_no, _pointVec[i]->_comp->getWidth()));
        }
        if (_pointVec[i]->_down.empty()) {
            _vsource->_up.push_back(make_pair(i, _vsource->_comp->height));
            _pointVec[i]->_down.push_back(make_pair(_vsource->_no, _vsource->_comp->height));
        }
        if (_pointVec[i]->_up.empty()) {
            _vsink->_down.push_back(make_pair(i, _pointVec[i]->_comp->getHeight()));
            _pointVec[i]->_up.push_back(make_pair(_vsink->_no, _pointVec[i]->_comp->getHeight()));
        }
    }

    /* put pseudo point in the last 4 position */
    _pointVec.push_back(_hsource);
    _pointVec.push_back(_hsink);
    _pointVec.push_back(_vsource);
    _pointVec.push_back(_vsink);
}

void CG::cal_leftMost()
{
    queue<Point*> Q;
    int inDegree[_pointVec.size()];
    memset(inDegree, 0, _pointVec.size()*sizeof(int));

    Q.push(_hsource);
    _hsource->_lm = _hsource->_comp->get_mid_x();

    while (!Q.empty()) {
        Point *p = Q.front();
        Q.pop();
        for (vector<pair<size_t, int>>::iterator it = p->_right.begin(); it != p->_right.end(); it++) {
            Point *right = _pointVec[it->first];
            if (right->_comp->type == FIXED) 
                right->_lm = right->_comp->get_mid_x();
            else if (p->_lm + it->second > right->_lm) 
                right->_lm = p->_lm + it->second;

            if (++inDegree[it->first] == right->_left.size())
                Q.push(right);
        }
    }
}
void CG::cal_rightMost()
{
    queue<Point*> Q;
    int inDegree[_pointVec.size()];
    memset(inDegree, 0, _pointVec.size()*sizeof(int));

    Q.push(_hsink);
    _hsink->_rm = _hsink->_comp->get_mid_x();

    while (!Q.empty()) {
        Point *p = Q.front();
        Q.pop();
        for (vector<pair<size_t, int>>::iterator it = p->_left.begin(); it != p->_left.end(); it++) {
            Point *left = _pointVec[it->first];
            if (left->_comp->type == FIXED) 
                left->_rm = left->_comp->get_mid_x();
            else if (p->_rm - it->second < left->_rm) 
                left->_rm = p->_rm - it->second;

            if (++inDegree[it->first] == left->_right.size())
                Q.push(left);
        }
    }
}
void CG::cal_downMost()
{
    queue<Point*> Q;
    int inDegree[_pointVec.size()];
    memset(inDegree, 0, _pointVec.size()*sizeof(int));

    Q.push(_vsource);
    _vsource->_dm = _vsource->_comp->get_mid_y();

    while (!Q.empty()) {
        Point *p = Q.front();
        Q.pop();
        for (vector<pair<size_t, int>>::iterator it = p->_up.begin(); it != p->_up.end(); it++) {
            Point* up = _pointVec[it->first];
            if (up->_comp->type == FIXED) 
                up->_dm = up->_comp->get_mid_y();
            else if (p->_dm + it->second > up->_dm) 
                up->_dm = p->_dm + it->second;

            if (++inDegree[it->first] == up->_down.size())
                Q.push(up);
        }
    }
}
void CG::cal_upMost()
{
    queue<Point*> Q;
    int inDegree[_pointVec.size()];
    memset(inDegree, 0, _pointVec.size()*sizeof(int));

    Q.push(_vsink);
    _vsink->_um = _vsink->_comp->get_mid_y();

    while (!Q.empty()) {
        Point *p = Q.front();
        Q.pop();
        for (vector<pair<size_t, int>>::iterator it = p->_down.begin(); it != p->_down.end(); it++) {
            Point *down = _pointVec[it->first];
            if (down->_comp->type == FIXED) 
                down->_um = down->_comp->get_mid_y();
            else if (p->_um - it->second < down->_um) 
                down->_um = p->_um - it->second;

            if (++inDegree[it->first] == down->_up.size())
                Q.push(down);
        }
    }
}

int CG::getHLongestPath(Point* start, Point *(&end))
{
    /* using topology sort */
    end = _hsink;

    queue<size_t> Q;
    int inDegree[_pointVec.size()];
    memset(inDegree, 0, _pointVec.size()*sizeof(int));

    int latestStart[_pointVec.size()];
    std::fill_n(latestStart, _pointVec.size(), start->_comp->get_ll_x());

    /* set start point */
    Q.push(start->_no);
    start->_criticalHParent = NULL;

    size_t index;
    Point *p;

    while (!Q.empty()) {

        index = Q.front();
        p = _pointVec[index];
        Q.pop();

        if (p->_comp->type == FIXED && p != _hsource && p != _hsink) {
            //check if macros touch the fixed block's left side 
            if (latestStart[index] > p->_comp->_llx) {
                end = p;
                return p->_comp->_llx - latestStart[index];
            }
            else
                latestStart[index] = p->_comp->_llx;
        }

        for (vector<pair<size_t, int>>::iterator it = p->_right.begin(); it != p->_right.end(); it++) {
            size_t rIndex = it->first;
            if (latestStart[index] + it->second >= latestStart[rIndex]) {
                
                latestStart[rIndex] = latestStart[index] + it->second;
                _pointVec[rIndex]->_criticalHParent = p;
            }

            if (++inDegree[rIndex] == _pointVec[rIndex]->_left.size() || p == start) 
                Q.push(rIndex);
        }
    }

    return _designRight - latestStart[_hsink->_no];
}

int CG::getVLongestPath(Point* start, Point *(&end))
{
    /* using topology sort */
    end = _vsink;

    queue<size_t> Q;
    int inDegree[_pointVec.size()];
    memset(inDegree, 0, _pointVec.size()*sizeof(int));

    int latestStart[_pointVec.size()];
    std::fill_n(latestStart, _pointVec.size(), start->_comp->get_ll_y());

    /* set start point */
    Q.push(start->_no);
    start->_criticalVParent = NULL;

    size_t index;
    Point *p;

    while (!Q.empty()) {

        index = Q.front();
        p = _pointVec[index];
        Q.pop();

        if (p->_comp->type == FIXED && p != _vsource && p != _vsink) {
            // check if macros touch the fixed block's buttom 
            if (latestStart[index] > p->_comp->_lly) {
                end = p;
                return p->_comp->_lly - latestStart[index];
            }
            else
                latestStart[index] = p->_comp->_lly;
        }

        for (vector<pair<size_t, int>>::iterator it = p->_up.begin(); it != p->_up.end(); it++) {
            size_t uIndex = it->first;
            if (latestStart[index] + it->second >= latestStart[uIndex]) {
                latestStart[uIndex] = latestStart[index] + it->second;
                _pointVec[uIndex]->_criticalVParent = p;
            }

            if (++inDegree[uIndex] == _pointVec[uIndex]->_down.size() || p == start)
                Q.push(uIndex);
        }
    }

    return _designUp - latestStart[_vsink->_no];
}

int CG::getHInverseLongestPath(Point* start, Point*& end) 
{
    /* using topology sort */
    end = _hsource;

    queue<size_t> Q;
    int inDegree[_pointVec.size()];
    memset(inDegree, 0, _pointVec.size()*sizeof(int));

    int latestStart[_pointVec.size()];
    std::fill_n(latestStart, _pointVec.size(), start->_comp->get_ll_x());

    /* set start point */
    Q.push(start->_no);

    size_t index;
    Point *p;

    while (!Q.empty()) {

        index = Q.front();
        p = _pointVec[index];
        Q.pop();

        if (p->_comp->type == FIXED && p != _hsource && p != _hsink) {
            //check if macros touch the fixed block's left side 
            if (latestStart[index] < p->_comp->_llx) {
                end = p;
                return latestStart[index] - p->_comp->_llx;
            }
            else
                latestStart[index] = p->_comp->_llx;
        }

        for (vector<pair<size_t, int>>::iterator it = p->_left.begin(); it != p->_left.end(); it++) {
            size_t lIndex = it->first;
            if (latestStart[index] - it->second <= latestStart[lIndex]) 
                latestStart[lIndex] = latestStart[index] - it->second;

            if (++inDegree[lIndex] == _pointVec[lIndex]->_right.size() || p == start) 
                Q.push(lIndex);
        }
    }
    
    return latestStart[_hsource->_no] - _designLeft;
}

int CG::getVInverseLongestPath(Point* start, Point*& end)
{
    /* using topology sort */
    end = _vsource;

    queue<size_t> Q;
    int inDegree[_pointVec.size()];
    memset(inDegree, 0, _pointVec.size()*sizeof(int));

    int latestStart[_pointVec.size()];
    std::fill_n(latestStart, _pointVec.size(), start->_comp->get_ll_y());

    /* set start point */
    Q.push(start->_no);

    size_t index;
    Point *p;

    while (!Q.empty()) {

        index = Q.front();
        p = _pointVec[index];
        Q.pop();

        if (p->_comp->type == FIXED && p != _vsource && p != _vsink) {
            // check if macros touch the fixed block's buttom 
            if (latestStart[index] < p->_comp->_lly) {
                end = p;
                return latestStart[index] - p->_comp->_lly;
            }
            else
                latestStart[index] = p->_comp->_lly;
        }

        for (vector<pair<size_t, int>>::iterator it = p->_down.begin(); it != p->_down.end(); it++) {
            size_t dIndex = it->first;
            if (latestStart[index] - it->second <= latestStart[dIndex]) {
                latestStart[dIndex] = latestStart[index] - it->second;
            }

            if (++inDegree[dIndex] == _pointVec[dIndex]->_up.size() || p == start)
                Q.push(dIndex);
        }
    }

    return latestStart[_vsource->_no] - _designDown;
}

void CG::deleteEdge(vector<pair<size_t, int>>& adjEdge, size_t no)
{
    vector<pair<size_t, int>>::iterator deleteEdge;
    for (vector<pair<size_t, int>>::iterator it = adjEdge.begin(); it != adjEdge.end();) {
        if (it->first == no)
            it = adjEdge.erase(it);
        else 
            ++it;
    }
    /*vector<pair<size_t, int>>::iterator deleteEdge = std::find_if(adjEdge.begin(), adjEdge.end(), 
            [&](std::pair<size_t, int> const & ref) 
            {return ref.first == no;});

    if (deleteEdge != adjEdge.end())
        adjEdge.erase(deleteEdge);
    else 
        cout << "cannot delete the edge" << endl;*/
}

void CG::deleteEdge(vector<pair<size_t, int>>& adjEdge, Point* p)
{
    vector<pair<size_t, int>>::iterator deleteEdge;
    for (vector<pair<size_t, int>>::iterator it = adjEdge.begin(); it != adjEdge.end();) {
        if (_pointVec[it->first] == p) {
            it = adjEdge.erase(it);
            break;
        }
        else 
            ++it;
    }
}

void CG::deletePoint(Point* p, bool graphType)
{
    cout << "delete point: " << p->_comp->name << endl;
    deletedPoint.push_back(p->_comp);

    /* cannot delete fixed block */
    if (p->_comp->type == FIXED) 
        return;

    /*if (graphType == H) {*/
        for (vector<pair<size_t, int>>::iterator i = p->_right.begin(); i != p->_right.end(); ++i) {

            Point *rightNeighbor = _pointVec[i->first];

            deleteEdge(rightNeighbor->_left, p);
        
            if (rightNeighbor->_left.empty() && rightNeighbor != _hsink) {
                _hsource->_right.push_back(make_pair(i->first, _hsource->_comp->getWidth()));
                rightNeighbor->_left.push_back(make_pair(_hsource->_no, _hsource->_comp->getWidth()));
            }
        }
        p->_right.clear();

        for (vector<pair<size_t, int>>::iterator i = p->_left.begin(); i != p->_left.end(); ++i) {

            Point *leftNeighbor = _pointVec[i->first];

            deleteEdge(leftNeighbor->_right, p);

            if (leftNeighbor->_right.empty() && leftNeighbor != _hsource) {
                _hsink->_left.push_back(make_pair(i->first, leftNeighbor->_comp->getWidth()));
                leftNeighbor->_right.push_back(make_pair(_hsink->_no, leftNeighbor->_comp->getWidth()));
            }
        }
        p->_left.clear();
    /*}
    else {*/
        for (vector<pair<size_t, int>>::iterator i = p->_up.begin(); i != p->_up.end(); ++i) {

            Point *upNeighbor = _pointVec[i->first];

            deleteEdge(upNeighbor->_down, p);

            if (upNeighbor->_down.empty() && upNeighbor != _vsink) {
                _vsource->_up.push_back(make_pair(i->first, _vsource->_comp->getHeight()));
                upNeighbor->_down.push_back(make_pair(_vsource->_no, _vsource->_comp->getHeight()));
            }
        }
        p->_up.clear();

        for (vector<pair<size_t, int>>::iterator i = p->_down.begin(); i != p->_down.end(); ++i) {

            Point* downNeighbor = _pointVec[i->first];

            deleteEdge(downNeighbor->_up, p);

            if (downNeighbor->_up.empty() && downNeighbor != _vsource) {
                _vsink->_down.push_back(make_pair(i->first, downNeighbor->_comp->getHeight()));
                downNeighbor->_up.push_back(make_pair(_vsink->_no, downNeighbor->_comp->getHeight()));
            }
        }
        p->_down.clear();
    /*}*/
}

void CG::swap(Point* a, Point* b)
{
    /*
    cout << "----- in swap function -----" << endl;

    // fixed macro, pseudopoint and boundary tile can't swap
    if (a->_comp->type == FIXED || b->_comp->type == FIXED) 
        return;
    
    vector<pair<size_t, int>>::iterator a_leftTo_b = std::find_if(a->_right.begin(), a->_right.end(), 
    [&](std::pair<size_t, int> const & ref) 
    {return _pointVec[ref.first] == b;});
    bool is_a_leftTo_b = a_leftTo_b != a->_right.end()? true : false;

    vector<pair<size_t, int>>::iterator b_leftTo_a = std::find_if(b->_right.begin(), b->_right.end(), 
    [&](std::pair<size_t, int> const & ref) 
    {return _pointVec[ref.first] == a;});
    bool is_b_leftTo_a = b_leftTo_a != b->_right.end()? true : false;

    vector<pair<size_t, int>>::iterator a_downTo_b = std::find_if(a->_up.begin(), a->_up.end(), 
    [&](std::pair<size_t, int> const & ref) 
    {return _pointVec[ref.first] == b;});
    bool is_a_downTo_b = a_downTo_b != a->_up.end()? true : false;

    vector<pair<size_t, int>>::iterator b_downTo_a = std::find_if(b->_up.begin(), b->_up.end(), 
    [&](std::pair<size_t, int> const & ref) 
    {return _pointVec[ref.first] == a;});
    bool is_b_downTo_a = b_downTo_a != b->_up.end()? true : false;

    if (is_a_leftTo_b || is_b_leftTo_a) {

        Point *left, *right;

        if (is_a_leftTo_b) {
            left = a;
            right = b;
        }else {
            left = b;
            right = a;
        }
        // left's left's right 
        for (vector<pair<size_t, int>>::iterator i = left->_left.begin(); i != left->_left.end(); ++i) {
            Point* leftNeighbor = _pointVec[i->first];

            // left's left don't need to change

            vector<pair<size_t, int>>::iterator updateEdge = std::find_if(leftNeighbor->_right.begin(), leftNeighbor->_right.end(), 
            [&](std::pair<size_t, int> const & ref) 
            {return _pointVec[ref.first] == left;});

            updateEdge->first = right->_no;
        }
        // left's right 
        for (vector<pair<size_t, int>>::iterator i = left->_right.begin(); i != left->_right.end(); ++i) {
            Point* rightNeighbor = _pointVec[i->first];

            if (i->first == right->_no)
                i->first = left->_no;
            i->second = right->_comp->width + _minSpace;

            vector<pair<size_t, int>>::iterator updateEdge = std::find_if(rightNeighbor->_left.begin(), rightNeighbor->_left.end(), 
            [&](std::pair<size_t, int> const & ref) 
            {return _pointVec[ref.first] == left;});

            updateEdge->first = right->_no;
            updateEdge->second = i->second;
        }
        // right's right 
        for (vector<pair<size_t, int>>::iterator i = right->_right.begin(); i != right->_right.end(); ++i) {
            Point* rightNeighbor = _pointVec[i->first];

            i->second = left->_comp->width + _minSpace;

            vector<pair<size_t, int>>::iterator updateEdge = std::find_if(rightNeighbor->_left.begin(), rightNeighbor->_left.end(), 
            [&](std::pair<size_t, int> const & ref) 
            {return _pointVec[ref.first] == right;});

            updateEdge->first = left->_no;
            updateEdge->second = i->second;
        }
        // right's left 
        for (vector<pair<size_t, int>>::iterator i = right->_left.begin(); i != right->_left.end(); ++i) {
            Point* leftNeighbor = _pointVec[i->first];

            if (i->first == left->_no)
                i->first = right->_no;
            i->second = right->_comp->width + _minSpace;

            vector<pair<size_t, int>>::iterator updateEdge = std::find_if(leftNeighbor->_right.begin(), leftNeighbor->_right.end(), 
            [&](std::pair<size_t, int> const & ref) 
            {return _pointVec[ref.first] == right;});

            updateEdge->first = left->_no;
            updateEdge->second = i->second;
        }

        std::swap(left->_left, right->_left);
        std::swap(left->_right, right->_right);

    }
    else if (is_a_downTo_b || is_b_downTo_a) {

    }
    */
}

void CG::move(Point* a, Point* b, bool graphType)
{
    /*
    cout << "move " << a->_comp->name << ' ' << b->_comp->name << endl;
    movedMacro.push_back(a->_comp);
    movedMacro.push_back(b->_comp);

    //if (a->_comp->type == FIXED || b->_comp->type == FIXED)
    //    return;

    bool duplicate = false;
    
    // a b have edge in hTCG 
    if (graphType == H) {

        Point *left = a;
        Point *right = b;

        cout << "before delete edge: " << "left is " << left->_comp->name << " right is " << right->_comp->name << endl;
        // delete (a, b) in hTCG 
        vector<pair<size_t, int>>::iterator deleteEdge1 = std::find_if(left->_right.begin(), left->_right.end(), 
        [&](std::pair<size_t, int> const & ref) 
        {return _pointVec[ref.first] == right;});
        if (deleteEdge1 == left->_right.end()) {
            duplicate = true;
            cout << "no Edge\n";
        } else {    
            left->_right.erase(deleteEdge1);
                if (left->_right.empty()) {
                left->_right.push_back(make_pair(_hsink->_no, left->_comp->getWidth() +_minSpace));
                _hsink->_left.push_back(make_pair(left->_no, left->_comp->getWidth() +_minSpace));
            }
        }

        vector<pair<size_t, int>>::iterator deleteEdge2 = std::find_if(right->_left.begin(), right->_left.end(), 
        [&](std::pair<size_t, int> const & ref) 
        {return _pointVec[ref.first] == left;});
        if (deleteEdge2 == right->_left.end()) {
            duplicate = true;
            cout << "no Edge\n";
        } else {
            right->_left.erase(deleteEdge2);
            if (right->_left.empty()) {
                _hsource->_right.push_back(make_pair(right->_no, _hsource->_comp->getWidth()));
                right->_left.push_back(make_pair(_hsource->_no, _hsource->_comp->getWidth()));
            }
        }

        if (!duplicate) {
            // add (a, b) in vTCG 
            int aHeight = a->_comp->get_ur_y() - b->_comp->get_ll_y();
            int bHeight = b->_comp->get_ur_y() - a->_comp->get_ll_y();

            Point *up, *down;

            if (aHeight >= 0 && bHeight >= 0) {
                if (aHeight < bHeight) {
                    up = b;
                    down = a;
                }else {
                    up = a;
                    down = b;
                }
            } else if (aHeight >= 0) {
                up = a;
                down = b;
            } else if (bHeight >= 0) {
                up = b;
                down = a;
            }

            if (down->_up.size() == 1 && down->_up[0].first == _vsink->_no) {
                down->_up.clear();
                deleteEdge(_vsink->_down, down->_no);
            }
            down->_up.push_back(make_pair(up->_no, down->_comp->getHeight() + _minSpace));

            if (up->_down.size() == 1 && up->_down[0].first == _vsource->_no) {
                up->_down.clear();
                deleteEdge(_vsource->_up, up->_no);        
            }
            up->_down.push_back(make_pair(down->_no, down->_comp->getHeight() + _minSpace));

            cout << "after add edge : " << "down is " << down->_comp->name << " up is " << up->_comp->name << endl;
        }

    } else {

        Point *down = a;
        Point *up = b;

        cout << "before delete edge: " << "down is " << down->_comp->name << " up is " << up->_comp->name << endl;
        // delete (a, b) in hTCG 
        vector<pair<size_t, int>>::iterator deleteEdge1 = std::find_if(down->_up.begin(), down->_up.end(), 
        [&](std::pair<size_t, int> const & ref) 
        {return _pointVec[ref.first] == up;});
        if(deleteEdge1 == down->_up.end()) {
            duplicate = true;
            cout << "no edge\n";
        } else {
            down->_up.erase(deleteEdge1);
            if (down->_up.empty()) {
                down->_up.push_back(make_pair(_vsink->_no, down->_comp->getHeight() + _minSpace));
                _vsink->_down.push_back(make_pair(down->_no, down->_comp->getHeight() + _minSpace));
            }
        }

        vector<pair<size_t, int>>::iterator deleteEdge2 = std::find_if(up->_down.begin(), up->_down.end(), 
        [&](std::pair<size_t, int> const & ref) 
        {return _pointVec[ref.first] == down;});
        if (deleteEdge2 == up->_down.end()) {
            duplicate = true;
            cout << "no edge\n";
        } else {
            up->_down.erase(deleteEdge2);
            if (up->_down.empty()) {
                up->_down.push_back(make_pair(_vsource->_no, _vsource->_comp->getHeight()));
                _vsource->_up.push_back(make_pair(up->_no, _vsource->_comp->getHeight()));
            }
        }

        if (!duplicate) {
            // add (a, b) in vTCG 
            int aWidth = a->_comp->get_ur_x() - b->_comp->get_ll_x();
            int bWidth = b->_comp->get_ur_x() - a->_comp->get_ll_x();

            Point *left, *right;

            if (aWidth >= 0 && bWidth >= 0) {
                if (aWidth < bWidth) {
                    right = b;
                    left = a;
                }else {
                    right = a;
                    left = b;
                }
            } else if (aWidth >= 0) {
                right = a;
                left = b;
            } else if (bWidth >= 0) {
                right = b;
                left = a;
            }

            if (left->_right.size() == 1 && left->_right[0].first == _hsink->_no) {
                left->_right.clear();
                deleteEdge(_hsink->_left, left->_no);
            }
            left->_right.push_back(make_pair(right->_no, left->_comp->getWidth() + _minSpace));

            if (right->_left.size() == 1 && right->_left[0].first == _hsource->_no) {
                right->_left.clear();
                deleteEdge(_hsource->_right, right->_no);
            }
            right->_left.push_back(make_pair(left->_no, left->_comp->getWidth() + _minSpace));

            cout << "after add edge : " << "left is " << left->_comp->name << " right is " << right->_comp->name << endl;
        }
    }
    */
}

void CG::solveLPbyCG()
{
    lprec *lp;
    int Ncol, *colno = NULL, j, ret = 0;
    REAL *row = NULL;

  /* We will build the model row by row
     So we start with creating a model with 0 rows and 2 columns */
    /* macro's x and y / macro's x and y 's absolute / macro's x and y 's initial position */
  Ncol = 2 * _pointVec.size() + 2 * _pointVec.size() + 2 * _pointVec.size();
  lp = make_lp(0, Ncol);
  if(lp == NULL)
    ret = 1; /* couldn't construct a new model... */

    /* let us name our variables. Not required, but can be useful for debugging */
    for (size_t i = 0; i < _pointVec.size(); ++i) {
        string colname1 = _pointVec[i]->_comp->getName() + "_x";
        string colname2 = _pointVec[i]->_comp->getName() + "_y";
        set_int(lp, 2 * i + 1, TRUE);
        set_int(lp, 2 * i + 2, TRUE);
        set_col_name(lp, 2 * i + 1, const_cast<char*>(colname1.c_str()));
        set_col_name(lp, 2 * i + 2, const_cast<char*>(colname2.c_str()));
    }
    for (size_t i = 0; i < _pointVec.size(); ++i) {
        size_t index = 2 * _pointVec.size() + 2 * i;
        string colname1 = "absx_" + _pointVec[i]->_comp->getName();
        string colname2 = "absy_" + _pointVec[i]->_comp->getName();
        set_int(lp, index + 1, TRUE);
        set_int(lp, index + 2, TRUE);
        set_col_name(lp, index + 1, const_cast<char*>(colname1.c_str()));
        set_col_name(lp, index + 2, const_cast<char*>(colname2.c_str()));
    }
    /* macro initial location (constant variable) */
    for (size_t i = 0; i < _pointVec.size(); ++i) {
        size_t index = 4 * _pointVec.size() + 2 * i;
        string colname1 = _pointVec[i]->_comp->getName() + "_inix";
        string colname2 = _pointVec[i]->_comp->getName() + "_iniy";
        set_int(lp, index + 1, TRUE);
        set_int(lp, index + 2, TRUE);
        set_col_name(lp, index + 1, const_cast<char*>(colname1.c_str()));
        set_col_name(lp, index + 2, const_cast<char*>(colname2.c_str()));
    }

    /* create space large enough for one row */
    colno = (int *) malloc(Ncol * sizeof(*colno));
    row = (REAL *) malloc(Ncol * sizeof(*row));

    set_add_rowmode(lp, TRUE);  /* makes building the model faster if it is done rows by row */

    /* construct first row (120 x + 210 y <= 15000) */
    /* right constraint*/
    for (size_t i = 0; i < _pointVec.size(); ++i) {
        if (i == _vsource->_no || i == _vsink->_no) 
            continue;
        if (_pointVec[i]->_comp->type == FIXED) {
            j = 0;
            colno[j] = 2 * i + 1;
            row[j++] = 1;
            add_constraintex(lp, j, row, colno, EQ, _pointVec[i]->_comp->get_ll_x());
        }
        for (vector<pair<size_t, int>>::iterator r = _pointVec[i]->_right.begin(); r != _pointVec[i]->_right.end(); ++r) {
            // right - i >= edge weight
            j = 0;

            colno[j] = 2 * (*r).first + 1;
            row[j++] = 1;

            colno[j] = 2 * i + 1;
            row[j++] = -1;

            // add the row to lpsolve 
            add_constraintex(lp, j, row, colno, GE, (*r).second);
        }
    }
    /* up constraint*/  
    for (size_t i = 0; i < _pointVec.size(); ++i) {
        if (i == _hsource->_no || i == _hsink->_no) 
            continue;
        if (_pointVec[i]->_comp->type == FIXED) {
            j = 0;
            colno[j] = 2 * i + 2;
            row[j++] = 1;
            add_constraintex(lp, j, row, colno, EQ, _pointVec[i]->_comp->get_ll_y());
        }
        for (vector<pair<size_t, int>>::iterator u = _pointVec[i]->_up.begin(); u != _pointVec[i]->_up.end(); ++u) {
            // up - i >= edge weight
            j = 0;

            colno[j] = 2 * (*u).first + 2;
            row[j++] = 1;

            colno[j] = 2 * i + 2;
            row[j++] = -1;

            // add the row to lpsolve 
            add_constraintex(lp, j, row, colno, GE, (*u).second);

        }
    }
    /* initial position setting*/
    for (size_t i = 0; i < _pointVec.size(); ++i) {

        size_t index = 4 * _pointVec.size() + 2 * i;
        
        j = 0;
        colno[j] = index + 1;
        row[j++] = 1;
        if (startBufferAreaLeg)
            add_constraintex(lp, j, row, colno, EQ, _pointVec[i]->_comp->_llx);
        else
            add_constraintex(lp, j, row, colno, EQ, _pointVec[i]->_comp->_originX);

        j = 0;
        colno[j] = index + 2;
        row[j++] = 1;
        if (startBufferAreaLeg)
            add_constraintex(lp, j, row, colno, EQ, _pointVec[i]->_comp->_lly);
        else 
            add_constraintex(lp, j, row, colno, EQ, _pointVec[i]->_comp->_originY);
    }

    /* absolute setting*/
    for (size_t i = 0; i < _pointVec.size(); ++i) {

        size_t absIndex = 2 * _pointVec.size() + 2 * i;
        size_t newPosIndex = 2 * i;
        size_t iniPosIndex = 4 * _pointVec.size() + 2 * i;

        j = 0;
        colno[j] = absIndex + 1;
        row[j++] = 1;
        colno[j] = newPosIndex + 1;
        row[j++] = -1;
        colno[j] = iniPosIndex + 1;
        row[j++] = 1;
        add_constraintex(lp, j, row, colno, GE, 0);
        j = 0;
        colno[j] = absIndex + 1;
        row[j++] = 1;
        colno[j] = newPosIndex + 1;
        row[j++] = 1;
        colno[j] = iniPosIndex + 1;
        row[j++] = -1;
        add_constraintex(lp, j, row, colno, GE, 0);

        j = 0;
        colno[j] = absIndex + 2;
        row[j++] = 1;
        colno[j] = newPosIndex + 2;
        row[j++] = -1;
        colno[j] = iniPosIndex + 2;
        row[j++] = 1;
        add_constraintex(lp, j, row, colno, GE, 0);
        j = 0;
        colno[j] = absIndex + 2;
        row[j++] = 1;
        colno[j] = newPosIndex + 2;
        row[j++] = 1;
        colno[j] = iniPosIndex + 2;
        row[j++] = -1;
        add_constraintex(lp, j, row, colno, GE, 0);
    }
  
    set_add_rowmode(lp, FALSE); /* rowmode should be turned off again when done building the model */

    /* minimize displacement */
    j = 0;
    for (size_t i = 0; i < _pointVec.size() - _boundaryComp.size() - 4; ++i) {
        size_t absIndex = 2 * _pointVec.size() + 2 * i;
        colno[j] = absIndex + 1;
        row[j++] = 1;
        colno[j] = absIndex + 2;
        row[j++] = 1;
    }

    /* set the objective in lpsolve */
    if(!set_obj_fnex(lp, j, row, colno))
      ret = 4;
  
    /* set the object direction to maximize */
    set_minim(lp);

    /* just out of curioucity, now show the model in lp format on screen */
    /* this only works if this is a console application. If not, use write_lp and a filename */
    //write_LP(lp, stdout);
    /* write_lp(lp, "model.lp"); */

    /* I only want to see important messages on screen while solving */
    //set_verbose(lp, IMPORTANT);

    /* Now let lpsolve calculate a solution */
    ret = solve(lp);

    /* a solution is calculated, now lets get some results */

    /* objective value */
    printf("Objective value: %f\n", get_objective(lp));

    /* variable values */
    get_variables(lp, row);
    /*for(j = 0; j < Ncol; j++){
        printf("%s: %f\n", get_col_name(lp, j + 1), row[j]);
    }*/

    for (size_t i = 0; i < _pointVec.size() - _boundaryComp.size() - 4; i++) {
        if (_compVec[i]->type == FIXED)
            continue;
        _compVec[i]->_llx = row[2 * i];
        _compVec[i]->_lly = row[2 * i + 1];
    }

    /* we are done now */
  

  /* free allocated memory */
  if(row != NULL)
    free(row);
  if(colno != NULL)
    free(colno);

  if(lp != NULL) {
    /* clean up such that all used memory by lpsolve is freed */
    delete_lp(lp);
  }
}

void CG::solveLPbyP2P()
{
    lprec *lp;
    int Ncol;
    int *colno = NULL;
    int c;
    int ret = 0;
    REAL *row = NULL;
 
    /* macro's x and y / macro's x and y 's absolute / macro's x and y 's initial position / z */
    int base = 2 * _compVec.size();
    int numOfZ = 4*((_compVec.size() * (_compVec.size() - 1)) / 2);
    Ncol = 2 * _compVec.size() + 2 * _compVec.size() + 2 * _compVec.size() + numOfZ;
    lp = make_lp(0, Ncol);

    if(lp == NULL)
        cout << "couldn't construct a new model\n"; /* couldn't construct a new model... */

    double Big_M = (double)_designRight;
    size_t zCount = 3 * base + 0;

    for (size_t i = 0; i < _compVec.size(); ++i) {

        size_t m1x = 2 * i + 1; // lower-left x of macro1
        size_t m1y = 2 * i + 2; // lower-left y of macro1

        /* set macro's buttom-left xy to integer */
        string mx = _compVec[i]->getName() + "_x";
        string my = _compVec[i]->getName() + "_y";
        set_int(lp, m1x, TRUE);
        set_int(lp, m1y, TRUE);
        set_col_name(lp, m1x, const_cast<char*>(mx.c_str()));
        set_col_name(lp, m1y, const_cast<char*>(my.c_str()));

        /* set abs(macro's buttom-left xy) to integer */
        string absmx = _compVec[i]->getName() + "_absx";
        string absmy = _compVec[i]->getName() + "_absy";
        set_int(lp, base + m1x, TRUE);
        set_int(lp, base + m1y, TRUE);
        set_col_name(lp, base + m1x, const_cast<char*>(absmx.c_str()));
        set_col_name(lp, base + m1y, const_cast<char*>(absmy.c_str()));

        /* set macro's initial position to integer */
        string inimx = _compVec[i]->getName() + "_inix";
        string inimy = _compVec[i]->getName() + "_iniy";
        set_int(lp, 2*base + m1x, TRUE);
        set_int(lp, 2*base + m1y, TRUE);
        set_col_name(lp, 2*base + m1x, const_cast<char*>(inimx.c_str()));
        set_col_name(lp, 2*base + m1y, const_cast<char*>(inimy.c_str()));
    }

    /* set z variable to binary */
    for (size_t i = 1; i <= numOfZ; ++i)
        set_binary(lp, zCount + i, TRUE);

    /* create space large enough for one row */
    colno = (int *) malloc(Ncol * sizeof(*colno));
    row = (REAL *) malloc(Ncol * sizeof(*row));
    set_add_rowmode(lp, TRUE);  /* makes building the model faster if it is done rows by row */

    for (size_t i = 0; i < _compVec.size() - 1; ++i) {
        for (size_t j = i + 1; j < _compVec.size(); ++j) {

            size_t m1x = 2 * i + 1; // lower-left x of macro1
            size_t m1y = 2 * i + 2; // lower-left y of macro1
            size_t m2x = 2 * j + 1; // lower-left x of macro2
            size_t m2y = 2 * j + 2; // lower-left y of macro2
            /*
            x2+w2≤x1+Mz1 
            x1+w1≤x2+Mz2
            y1+h1≤y2+Mz3
            y2+h2≤y1+Mz4
            z1+z2+z3+z4≤3
            */

            /* x1 - x2 + Mz1 >= w2 */
            c = 0;
            colno[c] = m1x; row[c++] = 1;
            colno[c] = m2x; row[c++] = -1;
            colno[c] = zCount + 1; row[c++] = Big_M;
            add_constraintex(lp, c, row, colno, GE, _compVec[j]->getWidth());
            /* x2 - x1 + Mz2 >= w1 */
            c = 0;
            colno[c] = m2x; row[c++] = 1;
            colno[c] = m1x; row[c++] = -1;
            colno[c] = zCount + 2; row[c++] = Big_M;
            add_constraintex(lp, c, row, colno, GE, _compVec[i]->getWidth());
            /* y2 - y1 + Mz3 >= h1 */
            c = 0;
            colno[c] = m2y; row[c++] = 1;
            colno[c] = m1y; row[c++] = -1;
            colno[c] = zCount + 3; row[c++] = Big_M;
            add_constraintex(lp, c, row, colno, GE, _compVec[i]->getHeight());
            /* y1 - y2 + Mz4 >= h2 */
            c = 0;
            colno[c] = m1y; row[c++] = 1;
            colno[c] = m2y; row[c++] = -1;
            colno[c] = zCount + 4; row[c++] = Big_M;
            add_constraintex(lp, c, row, colno, GE, _compVec[j]->getHeight());
            /* z1 + z2 + z3 + z4 <= 3 */
            c = 0;
            colno[c] = zCount + 1; row[c++] = 1;
            colno[c] = zCount + 2; row[c++] = 1;
            colno[c] = zCount + 3; row[c++] = 1;
            colno[c] = zCount + 4; row[c++] = 1;
            add_constraintex(lp, c, row, colno, LE, 3);

            zCount += 4;
        }
    }

    /* boundary limit constraints */
    for (size_t i = 0; i < _compVec.size(); ++i) {
        size_t m1x = 2 * i + 1; // lower-left x of macro1
        size_t m1y = 2 * i + 2; // lower-left y of macro1
        /* lower-left x > leftBoundary */
        c = 0;
        colno[c] = m1x; row[c++] = 1;
        add_constraintex(lp, c, row, colno, GE, _designLeft);
        /* lower-left y > downBoundary */
        c = 0;
        colno[c] = m1y; row[c++] = 1;
        add_constraintex(lp, c, row, colno, GE, _designDown);
        /* upper-right x > downBoundary */
        c = 0;
        colno[c] = m1x; row[c++] = 1;
        add_constraintex(lp, c, row, colno, LE, _designRight - _compVec[i]->getWidth());
        /* upper-right x > downBoundary */
        c = 0;
        colno[c] = m1y; row[c++] = 1;
        add_constraintex(lp, c, row, colno, LE, _designUp - _compVec[i]->getHeight());
    }

    /* initial position setting*/
    for (size_t i = 0; i < _compVec.size(); ++i) {

        size_t index = 2 * base + 2 * i;

        c = 0;
        colno[c] = index + 1;
        row[c++] = 1;
        add_constraintex(lp, c, row, colno, EQ, _compVec[i]->_originX);

        c = 0;
        colno[c] = index + 2;
        row[c++] = 1;
        add_constraintex(lp, c, row, colno, EQ, _compVec[i]->_originY);
    }

    /* absolute setting*/
    for (size_t i = 0; i < _compVec.size(); ++i) {

        size_t absIndex = base + 2 * i;
        size_t newPosIndex = 2 * i;
        size_t iniPosIndex = 2 * base + 2 * i;

        c = 0;
        colno[c] = absIndex + 1;
        row[c++] = 1;
        colno[c] = newPosIndex + 1;
        row[c++] = -1;
        colno[c] = iniPosIndex + 1;
        row[c++] = 1;
        add_constraintex(lp, c, row, colno, GE, 0);
        c = 0;
        colno[c] = absIndex + 1;
        row[c++] = 1;
        colno[c] = newPosIndex + 1;
        row[c++] = 1;
        colno[c] = iniPosIndex + 1;
        row[c++] = -1;
        add_constraintex(lp, c, row, colno, GE, 0);

        c = 0;
        colno[c] = absIndex + 2;
        row[c++] = 1;
        colno[c] = newPosIndex + 2;
        row[c++] = -1;
        colno[c] = iniPosIndex + 2;
        row[c++] = 1;
        add_constraintex(lp, c, row, colno, GE, 0);
        c = 0;
        colno[c] = absIndex + 2;
        row[c++] = 1;
        colno[c] = newPosIndex + 2;
        row[c++] = 1;
        colno[c] = iniPosIndex + 2;
        row[c++] = -1;
        add_constraintex(lp, c, row, colno, GE, 0);
    }

    /* constraints end */
    set_add_rowmode(lp, FALSE); /* rowmode should be turned off again when done building the model */

    /* set objective */
    c = 0;
    for (size_t i = 0; i < _compVec.size(); ++i) {
        size_t absIndex = base + 2 * i;
        colno[c] = absIndex + 1;
        row[c++] = 1;
        colno[c] = absIndex + 2;
        row[c++] = 1;
    }

    /* set the objective in lpsolve */
    if(!set_obj_fnex(lp, c, row, colno))
        ret = 4;

    /* set the object direction to maximize */
    set_minim(lp);

    /* just out of curioucity, now show the model in lp format on screen */
    /* this only works if this is a console application. If not, use write_lp and a filename */
    write_LP(lp, stdout);
    /* write_lp(lp, "model.lp"); */

    /* I only want to see important messages on screen while solving */
    set_verbose(lp, IMPORTANT);

    /* Now let lpsolve calculate a solution */
    ret = solve(lp);

    /* a solution is calculated, now lets get some results */

    /* objective value */
    printf("Objective value: %f\n", get_objective(lp));

    /* variable values */
    get_variables(lp, row);
    for(c = 0; c < Ncol; c++){
        printf("%s: %f\n", get_col_name(lp, c + 1), row[c]);
    }

    for (size_t i = 0; i < _compVec.size(); ++i) {
        /*if (_compVec[i]->type == FIXED)
            continue;*/
        _compVec[i]->_llx = row[2 * i];
        _compVec[i]->_lly = row[2 * i + 1];
    }

    /* we are done now */

    /* free allocated memory */
    if(row != NULL)
        free(row);
    if(colno != NULL)
        free(colno);

    if(lp != NULL) {
        /* clean up such that all used memory by lpsolve is freed */
        delete_lp(lp);
  }
}

vector<int> normalizeArea(const vector<Point*> &arr) {
    if (arr.empty())
        return {};

    vector<int> idx(arr.size()), ret(arr.size());

    /*iota(idx.begin(), idx.end(), 0);
    sort(idx.begin(), idx.end(),
         [&arr](int i, int j) { return arr[i]->_comp->getArea() < arr[j]->_comp->getArea(); });

    ret[idx[0]] = 1;
    for (size_t i = 1; i < arr.size(); ++i) {
        ret[idx[i]] = ret[idx[i - 1]] + (arr[idx[i]] != arr[idx[i - 1]]);
    }*/

    return ret;
}

vector<int> normalizeAspectRatio(const vector<Point*> &arr) {
    if (arr.empty())
        return {};

    vector<int> idx(arr.size()), ret(arr.size());

    /*iota(idx.begin(), idx.end(), 0);
    sort(idx.begin(), idx.end(), [&arr](int i, int j) { 
            double i_aspectRatio = arr[i]->_comp->getHeight() > arr[i]->_comp->getWidth() ?
                            (double)arr[i]->_comp->getHeight() / (double)arr[i]->_comp->getWidth() : 
                            (double)arr[i]->_comp->getWidth() / (double)arr[i]->_comp->getHeight();
            double j_aspectRatio = arr[j]->_comp->getHeight() > arr[j]->_comp->getWidth() ?
                            (double)arr[j]->_comp->getHeight() / (double)arr[j]->_comp->getWidth() :
                            (double)arr[j]->_comp->getWidth() / (double)arr[j]->_comp->getHeight();
            return i_aspectRatio < j_aspectRatio;
    });

    ret[idx[0]] = 1;
    for (size_t i = 1; i < arr.size(); ++i) {
        ret[idx[i]] = ret[idx[i - 1]] + (arr[idx[i]] != arr[idx[i - 1]]);
    }

    for (size_t i = 0; i < ret.size(); ++i)
        ret[i] *= 0.2;*/

    return ret;
}


std::vector<Point*>::iterator getBestCandidate(vector<Point*>& criticalList)
{
    /*vector<int> normalArea = normalizeArea(criticalList);
    vector<int> normalAspectRatio = normalizeAspectRatio(criticalList);

    cout << "area before normalized:\n";
    for (auto& i : criticalList)
        cout << i->_comp->getArea() << " ";
    cout << "\narea after normalized:\n";
    for (auto& i : normalArea)
        cout << i << " ";
    cout << endl;*/

    /*cout << "AspectRatio:\n";
    for (auto& i : normalAspectRatio)
        cout << i << " ";
    cout << endl;*/

    size_t smallest = 0;
    double cost = DBL_MAX;

    for (size_t nth = 0; nth < criticalList.size(); ++nth) {
        /*if (criticalList[nth]->_comp->getType() != FIXED
        && normalArea[nth] + normalAspectRatio[nth] < cost) {
            smallest = nth;
            cost = normalArea[nth] + normalAspectRatio[nth];
        }*/
        if (criticalList[nth]->_comp->getType() != FIXED
        && criticalList[nth]->_comp->getArea() < cost) {
            smallest = nth;
            cost = criticalList[nth]->_comp->getArea();
        }
    }

    return criticalList.begin() + smallest;
}


void CG::legalizeCriticalPath()
{
    int designWidth = _designRight - _designLeft;
    int designHeight = _designUp - _designDown;
    int longestHPath;
    int longestVPath;
    Point *h_end, *v_end;

    
    /* update longest path length */
    longestHPath = getHLongestPath(_hsource, h_end);
    cout << "longest H path: " << longestHPath << " / " << _designRight << endl;
    longestVPath = getVLongestPath(_vsource, v_end);
    cout << "longest V path: " << longestVPath << " / " << _designUp << endl;
    
    while (longestHPath < 0) {
        /* extract the macros in critical path */
        vector<Point*> criticalList;
        Point *current = h_end;
        cout << endl << longestHPath << " / " << _designRight << endl;
        cout << "criticalList H start: \n";
        while ((current = current->_criticalHParent)->_comp->getType() != FIXED) {
            cout << current->_comp->name << endl;
            criticalList.push_back(current);
        }
        cout << "criticalList H end: \n";

        // select the macro with smallest area
        std::vector<Point*>::iterator victimPoint = getBestCandidate(criticalList);
        /* if macro is bigger than threashold => move operation */
        if ((*victimPoint)->_comp->getArea() > (double)designWidth * (double)designHeight * 1.01 //0.01
            && criticalList.size() >= 2) { //A1/B10/C50/E1/E3/F6/o677480 A1/B10/C132/E1/E3/E2/o677498

            /* select reduction edge to move */
            Point *left, *right;
            if (victimPoint == criticalList.begin()) {
                left = *(victimPoint + 1);
                right = *victimPoint;
            }
            else if (victimPoint == criticalList.end() - 1) {
                left = *victimPoint;
                right = *(victimPoint - 1);
            }
            else if ((*(victimPoint + 1))->_comp->getHeight() < (*(victimPoint - 1))->_comp->getHeight()) {
                left = *(victimPoint + 1);
                right = *victimPoint;
            }
            else {
                left = *victimPoint;
                right = *(victimPoint - 1);
            }

            if (left->_comp->getType() != FIXED && right->_comp->getType() != FIXED)
                move(left, right, H);
            else {
                if (left->_comp->getType() == FIXED) 
                    deletePoint(right, H);
                else
                    deletePoint(left, H);
            }
        }
        /* else => delete operation */
        else 
            deletePoint(*victimPoint, H);

        longestHPath = getHLongestPath(_hsource, h_end);
    }

    while (longestVPath < 0) {

        /* extract the macros in critical path */
        vector<Point*> criticalList;
        Point *current = v_end;    // the start point of critical path
        cout << endl << longestVPath << " / " << _designUp << endl;
        cout << "criticalList V start: \n";
        while ((current = current->_criticalVParent)->_comp->getType() != FIXED) {
            cout << current->_comp->name << endl;
            criticalList.push_back(current);
        }
        cout << "criticalList V end: \n";


        std::vector<Point*>::iterator victimPoint = getBestCandidate(criticalList);

        if ((*victimPoint)->_comp->getArea() > (double)designWidth * (double)designHeight * 1.01 //0.01
            && criticalList.size() >= 2) {
            
            /* select reduction edge to move */
            Point *down, *up;
            if (victimPoint == criticalList.begin()) {
                down = *(victimPoint + 1);
                up = *victimPoint;
            }
            else if (victimPoint == criticalList.end() - 1) {
                down = *victimPoint;
                up = *(victimPoint - 1);
            }
            else if ((*(victimPoint + 1))->_comp->getWidth() < (*(victimPoint - 1))->_comp->getWidth()) {
                down = *(victimPoint + 1);
                up = *victimPoint;
            }
            else {
                down = *victimPoint;
                up = *(victimPoint - 1);
            }

            if (down->_comp->getType() != FIXED && up->_comp->getType() != FIXED)
                move(down, up, V);
            else {
                if (down->_comp->getType() == FIXED) 
                    deletePoint(up, V);
                else
                    deletePoint(down, V);
            }
        }
        else 
            deletePoint(*victimPoint, V);

        longestVPath = getVLongestPath(_vsource, v_end);
    }
    

    /* Debug */
    longestHPath = getHLongestPath(_hsource, h_end);
    cout << "final H critical length: " << longestHPath << " / " << _designRight << endl;
    longestVPath = getVLongestPath(_vsource, v_end);
    cout << "final V critical length: " << longestVPath << " / " << _designUp << endl;
}


void CG::optDeadSpace()
{
    cout << "----- in optDeadSpace Function -----" << endl;
    for (vector<Point*>::iterator it = _pointVec.begin(); it != _pointVec.end(); ++it) {
        Point* point = *it;
        for (vector<pair<size_t, int>>::iterator right = point->_right.begin(); right != point->_right.end(); ++right) {
            int longestHPath = getHLongestPath(_pointVec[right->first], _hsink);
            int longestVPath = getVLongestPath(_pointVec[right->first], _vsink);
            if (optimizeBufferArea(point, _pointVec[right->first], H) &&
            longestHPath + _pwc < _designRight /*&& longestVPath + _pwc <= _designUp*/) {
                cout << longestHPath << " + " << _pwc << " = " << longestHPath + _pwc << " <= " << _designRight << endl;
                if (point->_comp->getType() != FIXED)
                    right->second = point->_comp->getWidth() + _pwc;
            }
        }
    }
    for (vector<Point*>::iterator it = _pointVec.begin(); it != _pointVec.end(); ++it) {
        Point* point = *it;
        for (vector<pair<size_t, int>>::iterator up = point->_up.begin(); up != point->_up.end(); ++up) {
            int longestVPath = getVLongestPath(_pointVec[up->first], _vsink);
            int longestHPath = getHLongestPath(_pointVec[up->first], _hsink);
            if (optimizeBufferArea(point, _pointVec[up->first], V) &&
            longestVPath + _pwc < _designUp /*&& longestHPath + _pwc <= _designRight*/) {
                cout << longestVPath << " + " << _pwc << " = " << longestVPath + _pwc << " <= " << _designUp << endl;
                if (point->_comp->getType() != FIXED)
                    up->second = point->_comp->getHeight() + _pwc;
            }
        }
    }
}


void CG::stickToBoundary()
{
    for (vector<pair<size_t, int>>::iterator it = _hsource->_right.begin(); it != _hsource->_right.end(); ++it) {

        Component* macro = _pointVec[it->first]->_comp;
        Component* boundary = _hsource->_comp;
        int distance = macro->get_ll_x() - boundary->get_ll_x();
        if (distance < _pwc)
            macro->_llx = boundary->get_ll_x();
    }
    for (vector<pair<size_t, int>>::iterator it = _hsink->_left.begin(); it != _hsink->_left.end(); ++it) {

        Component* macro = _pointVec[it->first]->_comp;
        Component* boundary = _hsink->_comp;
        int distance = boundary->get_ll_x() - macro->get_ur_x();
        if (distance < _pwc)
            macro->_llx = boundary->get_ll_x() - macro->getWidth();
    }
    for (vector<pair<size_t, int>>::iterator it = _vsource->_up.begin(); it != _vsource->_up.end(); ++it) {

        Component* macro = _pointVec[it->first]->_comp;
        Component* boundary = _vsource->_comp;
        int distance = macro->get_ll_y() - boundary->get_ll_y();
        if (distance < _pwc)
            macro->_lly = boundary->get_ll_y();
    }
    for (vector<pair<size_t, int>>::iterator it = _vsink->_down.begin(); it != _vsink->_down.end(); ++it) {

        Component* macro = _pointVec[it->first]->_comp;
        Component* boundary = _vsink->_comp;
        int distance = boundary->get_ll_y() - macro->get_ur_y();
        if (distance < _pwc)
            macro->_lly = boundary->get_ll_y() - macro->getHeight();
    }
}

void CG::legalize()
{
    clock_t start, end;
    double lp_runtime = 0.0;

    do {
        start = clock();

        /* static*/
        lastDeletedPoint = deletedPoint;
        /* reset source*/
        deletedPoint.clear();
        /* initialize */
        initialize();
        /* build TCG */
        buildGraph();
        /* delete point to make TCG fit in the boundary */
        legalizeCriticalPath();
        /* linear programming solver */
        solveLPbyCG();
        stickToBoundary();

        /* reset source*/
        delete _hsource;
        _hsource = NULL;
        delete _vsource;
        _vsource = NULL;
        delete _hsink;
        _hsink = NULL;
        delete _vsink;
        _vsink = NULL;

        end = clock();
        lp_runtime += double(end-start) / CLOCKS_PER_SEC;

    } while(lastDeletedPoint != deletedPoint && lp_runtime < 180.0);

}

void CG::optBufferArea()
{
    /* reset source*/
    delete _hsource;
    _hsource = NULL;
    delete _vsource;
    _vsource = NULL;
    delete _hsink;
    _hsink = NULL;
    delete _vsink;
    _vsink = NULL;

    startBufferAreaOpt = true;
    /* optimize dead space*/
    //initialize
    initialize();
    // build TCG 
    buildGraph();
    deletedPoint.clear();
    // delete point to make TCG fit in the boundary
    legalizeCriticalPath();
    // linear programming solver 
    solveLPbyCG();
    stickToBoundary();
    startBufferAreaOpt = false;
}

void CG::leaveSpaceForBufferArea(vector<Component*>& illegal)
{
    cout << "\n----- leaveSpaceForBufferArea -----\n";
    int designWidth = _designRight - _designLeft;
    int designHeight = _designUp - _designDown;
    int longestHPath;
    int longestVPath;
    int longestHInversePath;
    int longestVInversePath;
    Point *h_end, *v_end;

    for (vector<Component*>::iterator it = illegal.begin(); it != illegal.end(); ++it) {
        size_t index = std::find(_compVec.begin(), _compVec.end(), *it) - _compVec.begin();
        cout << _pointVec[index]->_comp->name << endl;
        longestHPath = getHLongestPath(_pointVec[index], h_end);
        longestVPath = getVLongestPath(_pointVec[index], v_end);
        longestHInversePath = getHInverseLongestPath(_pointVec[index], h_end);
        longestVInversePath = getVInverseLongestPath(_pointVec[index], v_end);
        cout << "longestHPath source: " << longestHPath << " / " << _designRight << endl;
        cout << "longestVPath source: " << longestVPath << " / " << _designUp << endl;
        cout << "longestHInversePath source: " << longestHInversePath << " / " << _designLeft << endl;
        cout << "longestVInversePath source: " << longestVInversePath << " / " << _designDown << endl;

        int bigestSlack = longestHPath;
        if (longestVPath > bigestSlack)
            bigestSlack = longestVPath;
        if (longestHInversePath > bigestSlack)
            bigestSlack = longestHInversePath;
        if (longestVInversePath > bigestSlack)
            bigestSlack = longestVInversePath;
        
        if (bigestSlack == longestHPath) {
            for (vector<pair<size_t, int>>::iterator r = _pointVec[index]->_right.begin(); r != _pointVec[index]->_right.end(); ++r) {
                int overlapLength = std::max(0, 
                min(_pointVec[index]->_comp->get_ur_y(), _pointVec[r->first]->_comp->get_ur_y()) - max(_pointVec[index]->_comp->get_ll_y(), _pointVec[r->first]->_comp->get_ll_y()));
                int dis = _pointVec[r->first]->_comp->get_ll_x() - _pointVec[index]->_comp->get_ur_x();
                if (overlapLength > 0 && dis < _pwc) {
                    r->second = _pointVec[index]->_comp->getWidth() + _pwc;
                }
            }
        }
        else if (bigestSlack == longestVPath) {
            for (vector<pair<size_t, int>>::iterator u = _pointVec[index]->_up.begin(); u != _pointVec[index]->_up.end(); ++u) {
                int overlapLength = std::max(0, 
                min(_pointVec[index]->_comp->get_ur_x(), _pointVec[u->first]->_comp->get_ur_x()) - max(_pointVec[index]->_comp->get_ll_x(), _pointVec[u->first]->_comp->get_ll_x()));
                int dis = _pointVec[u->first]->_comp->get_ll_y() - _pointVec[index]->_comp->get_ur_y();
                if (overlapLength > 0 && dis < _pwc) {
                    u->second = _pointVec[index]->_comp->getHeight() + _pwc;
                }
            }
        }
        else if (bigestSlack == longestHInversePath) {
            for (vector<pair<size_t, int>>::iterator l = _pointVec[index]->_left.begin(); l != _pointVec[index]->_left.end(); ++l) {
                for (vector<pair<size_t, int>>::iterator lr = _pointVec[l->first]->_right.begin(); lr != _pointVec[l->first]->_right.end(); ++lr) {
                    if (lr->first == index) {
                        int overlapLength = std::max(0, 
                        min(_pointVec[l->first]->_comp->get_ur_y(), _pointVec[index]->_comp->get_ur_y()) - max(_pointVec[l->first]->_comp->get_ll_y(), _pointVec[index]->_comp->get_ll_y()));
                        int dis = _pointVec[index]->_comp->get_ll_x() - _pointVec[l->first]->_comp->get_ur_x();
                        if (overlapLength > 0 && dis < _pwc) {
                            lr->second = _pointVec[l->first]->_comp->getWidth() + _pwc;
                        }
                        break;
                    }
                }
            }
        }
        else if (bigestSlack == longestVInversePath) {
            for (vector<pair<size_t, int>>::iterator d = _pointVec[index]->_down.begin(); d != _pointVec[index]->_down.end(); ++d) {
                for (vector<pair<size_t, int>>::iterator du = _pointVec[d->first]->_up.begin(); du != _pointVec[d->first]->_up.end(); ++du) {
                    if (du->first == index) {
                        int overlapLength = std::max(0, 
                        min(_pointVec[d->first]->_comp->get_ur_x(), _pointVec[index]->_comp->get_ur_x()) - max(_pointVec[d->first]->_comp->get_ll_x(), _pointVec[index]->_comp->get_ll_x()));
                        int dis = _pointVec[index]->_comp->get_ll_y() - _pointVec[d->first]->_comp->get_ur_y();
                        if (overlapLength > 0 && dis < _pwc) {
                            du->second = _pointVec[d->first]->_comp->getHeight() + _pwc;
                        }
                        break;
                    }
                }
            }
        }
    }

    // update longest path length 
    longestHPath = getHLongestPath(_hsource, h_end);
    cout << "longest H path: " << longestHPath << " / " << _designRight << endl;
    longestVPath = getVLongestPath(_vsource, v_end);
    cout << "longest V path: " << longestVPath << " / " << _designUp << endl;
    
    while (longestHPath < 0) {
        // extract the macros in critical path 
        vector<Point*> criticalList;
        Point *current = h_end;
        cout << endl << longestHPath << " / " << _designRight << endl;
        cout << "criticalList H start: \n";
        while ((current = current->_criticalHParent)->_comp->getType() != FIXED) {
            cout << current->_comp->name << endl;
            criticalList.push_back(current);
        }
        cout << "criticalList H end: \n";

        // select the macro with smallest area
        std::vector<Point*>::iterator victimPoint = getBestCandidate(criticalList);
        deletePoint(*victimPoint, H);

        longestHPath = getHLongestPath(_hsource, h_end);
    }

    while (longestVPath < 0) {

        // extract the macros in critical path 
        vector<Point*> criticalList;
        Point *current = v_end;    // the start point of critical path
        cout << endl << longestVPath << " / " << _designUp << endl;
        cout << "criticalList V start: \n";
        while ((current = current->_criticalVParent)->_comp->getType() != FIXED) {
            cout << current->_comp->name << endl;
            criticalList.push_back(current);
        }
        cout << "criticalList V end: \n";

        std::vector<Point*>::iterator victimPoint = getBestCandidate(criticalList);
        deletePoint(*victimPoint, V);
        longestVPath = getVLongestPath(_vsource, v_end);
    }
    

    /* Debug */
    /*longestHPath = getHLongestPath(_hsource, h_end);
    cout << "final H critical length: " << longestHPath << " / " << _designRight << endl;
    longestVPath = getVLongestPath(_vsource, v_end);
    cout << "final V critical length: " << longestVPath << " / " << _designUp << endl;*/
}

void CG::legalizeBufferConstraint(vector<Component*>& illegal)
{
    startBufferAreaLeg = true;

    initialize();
    // build TCG 
    buildGraph();
    deletedPoint.clear();
    // delete point to make TCG fit in the boundary 
    leaveSpaceForBufferArea(illegal);
    // linear programming solver 
    solveLPbyCG();

    /* reset */
    delete _hsource;
    _hsource = NULL;
    delete _vsource;
    _vsource = NULL;
    delete _hsink;
    _hsink = NULL;
    delete _vsink;
    _vsink = NULL;

    startBufferAreaLeg = false;
}
