#include "preprocessor.h"



/*
 *
 *
 *
 */
void Preprocessor::run(){
    /* build the bounding box of the design */
    buildBoundingRect();
    /* build the tile plane with _boundingRect */
    _plane = new Plane(_boundingRect);
    /* build the boundary tiles in _plane */
    buildBoundaryTile();
}




/*
 *
 * build bounding box for the overall design using boundary edges.
 *
 */
void Preprocessor::buildBoundingRect(){
    /* initial _boundingRect with first bound */
    /* direction of bound is virtical */
    int min = MIN(_boundVec[0]->start, _boundVec[0]->end);
    int max = MAX(_boundVec[0]->start, _boundVec[0]->end);
    int pos = _boundVec[0]->getPos();

    if(_boundVec[0]->getDirection() == V){
        _boundingRect.set4Side(pos, min, pos, max);
    }
    /* direction of bound is horizontal */
    else{
        _boundingRect.set4Side(min, pos, max, pos);
    }

    /* traverse each bound in _boundVec */
    for(size_t i = 1; i < _boundVec.size(); ++i){
        pos = _boundVec[i]->getPos();
        min = MIN(_boundVec[i]->start, _boundVec[i]->end);
        max = MAX(_boundVec[i]->start, _boundVec[i]->end);

        /* direction of bound is virtical */
        if(_boundVec[i]->getDirection() == V){
            if(pos < _boundingRect.getLeft())
                _boundingRect.setLeft(pos);
            else if(pos > _boundingRect.getRight())
                _boundingRect.setRight(pos);

            if(min < _boundingRect.getBottom())
                _boundingRect.setBottom(min);

            if(max > _boundingRect.getTop())
                _boundingRect.setTop(max);
        }
        /* direction of bound is horizontal */
        else{
            if(min < _boundingRect.getLeft())
                _boundingRect.setLeft(min);

            if(max > _boundingRect.getRight())
                _boundingRect.setRight(max);

            if(pos < _boundingRect.getBottom())
                _boundingRect.setBottom(pos);
            else if(pos > _boundingRect.getTop())
                _boundingRect.setTop(pos);
        }
        
    }
}



/*
 *
 * build boundary tile on _plane when the boundary shape is rectilinear.
 *
 */
void Preprocessor::buildBoundaryTile(){
    cout << "\n----- In buildBoundaryTile function -----\n";

    size_t boundVecSize = _boundVec.size();

    /*vector<pair<int, int>> eventPoint;

    for(size_t i = 0; i < boundVecSize; ++i){
        int direction = _boundVec[i]->getRelation();
        if(direction == UP || direction == DOWN){
            eventPoint.push_back(pair<int, int>(_boundVec[i]->getPos(), i));
        }            
    }

    std::sort(eventPoint.begin(), eventPoint.end());

    for(size_t i = 0; i < eventPoint.size(); ++i){
        int boundIndex = eventPoint[i].second;
        int prevBoundIndex = (boundIndex - 1 >= 0) ? boundIndex - 1 : boundVecSize - 1;
        int nextBoundIndex = (boundIndex + 1 < boundVecSize) ? boundIndex + 1 : 0;

        Bound *bound = _boundVec[boundIndex];
        Bound *prevBound = _boundVec[prevBoundIndex];
        Bound *nextBound = _boundVec[nextBoundIndex];

        cout << "current bound: " << bound->start << " " << bound->end << " " << bound->pos << endl;
        cout << "prev bound: " << prevBound->start << " " << prevBound->end << " " << prevBound->pos << endl;
        cout << "next bound: " << nextBound->start << " " << nextBound->end << " " << nextBound->pos << endl;
    }
    */
    int boundaryTop = _boundingRect.getTop();
    int boundaryBottom = _boundingRect.getBottom();

    for(size_t i = 0; i < _boundVec.size(); ++i){
        /* direction of bound is horizontal */
        if(_boundVec[i]->getDirection() == H){
            int start = _boundVec[i]->start;
            int end = _boundVec[i]->end;
            int pos = _boundVec[i]->getPos();
            
            /* build upper boundary tile */
            if(start < end && pos < boundaryTop){
                Rectangle rect(start, pos, end, boundaryTop);
                CornerStitch::insertTile(&rect, _plane);

                // add boundary component for TCG
                Component* newBoundaryComp = new Component(rect.getLeft(), rect.getBottom(), rect.getWidth(), rect.getHeight());
                newBoundaryComp->name = "boundaryComp";
                newBoundaryComp->type = FIXED;
                _boundaryComp.push_back(newBoundaryComp);
            }
            
            /* build lower boundary tile */
            if(start > end && pos > boundaryBottom){
                Rectangle rect(end, boundaryBottom, start, pos);
                CornerStitch::insertTile(&rect, _plane);

                // add boundary component for TCG
                Component* newBoundaryComp = new Component(rect.getLeft(), rect.getBottom(), rect.getWidth(), rect.getHeight());
                newBoundaryComp->name = "boundaryComp";
                newBoundaryComp->type = FIXED;
                _boundaryComp.push_back(newBoundaryComp);
            }
        }
    }
}

void 
Preprocessor::insertBoundaryCompnent(int lower_x, int lower_y, int width, int height, vector<Component*>& vec)
{
	Component *add_fs = new Component(lower_x, lower_y, width, height);
	add_fs->type = FIXED;
	add_fs->name = "boundaryComp";
	
	//cout << "insert: " << add_fs->get_ll_x() << " " << add_fs->get_ll_y() << " " << add_fs->getWidth() << " " << add_fs->getHeight() << endl;
	if (width == 0 || height == 0)
		return;
	
	vec.push_back(add_fs);
}

void
Preprocessor::buildInsideBoundaryComponent(vector<Component*>& insideComp)
{
    /* initialization*/
	priority_queue<Event> boundary;
	IntervalTree line;

	for (vector<Bound*>::iterator it = _boundVec.begin(); it != _boundVec.end(); ++it) {

		int relation = (*it)->getRelation();
        // don't load left and right boudary
		if (relation == LEFT || relation == RIGHT) 
            continue;
		
		Component *boundaryToComp = new Component(*it);
        // down boundary => begin = false, up boundary => begin = true
		Event boundaryEvent(boundaryToComp, relation, boundaryToComp->get_ll_y(), true);
		boundaryEvent.bound = *it;
		boundary.push(boundaryEvent);
	}

	while (!boundary.empty()) {

		Node* node;
		Event currentEvent = boundary.top();

		node = new Node(boundary.top().comp, V);
		currentEvent = boundary.top();
		boundary.pop();
		
		if (currentEvent.begin == true) {
			Node* target = line.get_interval(line.root, node, V);

			int width = target->high - target->low;
			int height = currentEvent.pos - target->begin;

			insertBoundaryCompnent(target->low, target->begin, width, height, insideComp);

			Node *left_interval = NULL;
			Node *right_interval = NULL;

			if (node->getLow() != target->getLow()) {
				left_interval = new Node(target->low, node->low);
				left_interval->begin = currentEvent.pos;
			}
			if (node->getHigh() != target->getHigh()) {
				right_interval = new Node(node->high, target->high);
				right_interval->begin = currentEvent.pos;
			}

			line.root = line.deleteNode(line.root, target);

			DEBUG("first\n");
			if (left_interval != NULL) 
				line.root = line.insert(line.root, left_interval);

			if (right_interval != NULL) 
				line.root = line.insert(line.root, right_interval);
		}
		else {
			Node* merge = new Node();

			Node *left_neighbor = line.find_left_adjacent_interval(line.root, node->low);
			if (left_neighbor != NULL) {
				// merge left left freespace
				merge->low = left_neighbor->low;

				// add left freespace
				int width = left_neighbor->high - left_neighbor->low;
				int height = currentEvent.pos - left_neighbor->begin;

				insertBoundaryCompnent(left_neighbor->low, left_neighbor->begin, width, height, insideComp);

				line.root = line.deleteNode(line.root, left_neighbor);
			}
			else
				merge->low = node->low;

			//line.print_infix(line.root);

			// add right freespace
			DEBUG("delete right\n");
			Node *right_neighbor = line.find_right_adjacent_interval(line.root, node->high);
			if (right_neighbor != NULL) {
				// merge left right freespace
				merge->high = right_neighbor->high;

				int width = right_neighbor->high - right_neighbor->low;
				int height = currentEvent.pos - right_neighbor->begin;
				
				insertBoundaryCompnent(right_neighbor->low, right_neighbor->begin, width, height, insideComp);

				line.root = line.deleteNode(line.root, right_neighbor);
			}
			else
				merge->high = node->high;

			merge->pos = (merge->low + merge->high) / 2.0;
			merge->max = merge->high;
			merge->begin = currentEvent.pos;

			line.root = line.insert(line.root, merge);
			//line.print_infix(line.root);
		}
	}
}

void
Preprocessor::buildOutsideBoundaryComponent(vector<Component*>& insideComp)
{
	/* initialize */
	priority_queue<Event> event;
	priority_queue<Event> boundary;
	IntervalTree line;

    /* load lower bound of boundary*/
	Component* start = new Component(_parser.minOfWidth, _parser.minOfHeight, 
									(_parser.maxOfWidth - _parser.minOfWidth), 0);
	Event startEvent(start, false, start->get_ll_y(), true);
	boundary.push(startEvent);

	/* load upper bound of boundary*/
	Component* end = new Component(_parser.minOfWidth, _parser.maxOfHeight, 
									(_parser.maxOfWidth - _parser.minOfWidth), 0);
	Event endEvent(end, true, end->get_ll_y(), true);
	boundary.push(endEvent);

    /* load component inside the boundary*/
	for (vector<Component*>::iterator i = insideComp.begin(); i != insideComp.end(); i++) {
		event.push(Event(*i, true, (*i)->get_ll_y()));
		event.push(Event(*i, false, (*i)->get_ur_y()));
	}

	/* sweep line algorithm*/
	while (!boundary.empty()) {

		Node* node;
		Event currentEvent;

		/* if only boundary remaining */
		if (event.empty()) {
			node = new Node(boundary.top().comp, V);
			currentEvent = boundary.top();
			boundary.pop();
		}
		/* if only macro remaining */
		else if (boundary.empty()) {
			// won't happen
		}
		/* load low boundary */
		/* load high boundary */
		else if ((boundary.top().pos <= event.top().pos && boundary.top().begin == false) 
		|| (boundary.top().pos < event.top().pos && boundary.top().begin == true)) {
			node = new Node(boundary.top().comp, V);
			currentEvent = boundary.top();
			boundary.pop();
		}
		/* load macro */
		else {
			node = new Node(event.top().comp, V);
			currentEvent = event.top();
			event.pop();
		}
		
		if (currentEvent.begin == true) {
			Node* target = line.get_interval(line.root, node, V);

			int width = target->high - target->low;
			int height = currentEvent.pos - target->begin;

			insertBoundaryCompnent(target->low, target->begin, width, height, _boundaryComp);

			Node *left_interval = NULL;
			Node *right_interval = NULL;

			if (node->getLow() != target->getLow()) {
				left_interval = new Node(target->low, node->low);
				left_interval->begin = currentEvent.pos;
			}
			if (node->getHigh() != target->getHigh()) {
				right_interval = new Node(node->high, target->high);
				right_interval->begin = currentEvent.pos;
			}

			line.root = line.deleteNode(line.root, target);

			DEBUG("first\n");
			if (left_interval != NULL) 
				line.root = line.insert(line.root, left_interval);

			if (right_interval != NULL) 
				line.root = line.insert(line.root, right_interval);
		}
		else {
			Node* merge = new Node();

			Node *left_neighbor = line.find_left_adjacent_interval(line.root, node->low);
			if (left_neighbor != NULL) {
				// merge left left freespace
				merge->low = left_neighbor->low;

				// add left freespace
				int width = left_neighbor->high - left_neighbor->low;
				int height = currentEvent.pos - left_neighbor->begin;

				insertBoundaryCompnent(left_neighbor->low, left_neighbor->begin, width, height, _boundaryComp);

				line.root = line.deleteNode(line.root, left_neighbor);
			}
			else
				merge->low = node->low;

			//line.print_infix(line.root);

			// add right freespace
			Node *right_neighbor = line.find_right_adjacent_interval(line.root, node->high);
			if (right_neighbor != NULL) {
				// merge left right freespace
				merge->high = right_neighbor->high;

				int width = right_neighbor->high - right_neighbor->low;
				int height = currentEvent.pos - right_neighbor->begin;
				
				insertBoundaryCompnent(right_neighbor->low, right_neighbor->begin, width, height, _boundaryComp);

				line.root = line.deleteNode(line.root, right_neighbor);
			}
			else
				merge->high = node->high;

			merge->pos = (merge->low + merge->high) / 2.0;
			merge->max = merge->high;
			merge->begin = currentEvent.pos;

			line.root = line.insert(line.root, merge);
			//line.print_infix(line.root);
		}
	}
}

void 
Preprocessor::buildBoundaryComponent()
{   
    /* seperate boundary(inside) to the multiple rectangles*/
	vector<Component*> insideComp;
	buildInsideBoundaryComponent(insideComp);
    /* fill in boundary(outside) with multiple rectangles*/
	buildOutsideBoundaryComponent(insideComp);
}