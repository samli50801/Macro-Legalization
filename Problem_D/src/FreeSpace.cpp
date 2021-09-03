#include "FreeSpace.h"

bool
FreeSpace::buffer_area_reservation(const double baredc)
{
	/*Component *temp = new Component(x - width, y, width, height);

	unordered_set<Component*>::iterator start = free_space.upper_bound(temp);

	if (start == free_space.end())
		return false;

	//DEBUG(endl <<temp->get_ur_x() <<endl << (*start)->get_ur_x() << endl);
	//for (auto i = free_space.begin(); i != free_space.end(); i++) {
		DEBUG((*i)->get_ur_x() << " ");
	//}

	bool x_overlap, y_overlap;
	for (; start != free_space.end(); start++) {
		//fGe
		x_overlap = fGe((width + (*start)->width) / 2.0, fabs(temp->x - (*start)->x)) ? true : false;
		y_overlap = fGe((height + (*start)->height) / 2.0, fabs(temp->y - (*start)->y)) ? true : false;
		if (x_overlap && y_overlap) {
			//(*start)->type = PLACED; // just for drawing 
			return true;
		}
	}*/

	return false;
}

void
FreeSpace::insertFreeSpace(int lower_x, int lower_y, int width, int height, bool type)
{
	Component *add_fs = new Component(lower_x, lower_y, width, height);
	add_fs->type = FIXED;
	add_fs->name = "ADDED";
	/*DEBUG("add freespace: " << add_fs->get_ll_x() << " " << add_fs->get_ll_x() + add_fs->width << " " << add_fs->get_ll_y() << " " << add_fs->height << endl;)*/

	if (width == 0 || height == 0)
		return;
	if (type == V) {
		if (add_fs->width < parser._pwc * parser.dbuPerMicron) 
			not_free_space.insert(add_fs);
		/*else 
			free_space.insert(add_fs);*/
	}
	else if (type == H) {
		if (add_fs->height < parser._pwc * parser.dbuPerMicron)
			not_free_space.insert(add_fs);
		else 
			free_space.insert(add_fs);
	}
}

bool ifHorizontalOverlap(Component* a, Component* b) {
	if (a->get_ur_x() < b->get_ll_x() || b->get_ur_x() < a->get_ll_x())
		return false;
	return true;
}
bool ifVerticalOverlap(Component* a, Component* b) {
	if (a->get_ur_y() < b->get_ll_y() || b->get_ur_y() < a->get_ll_y())
		return false;
	return true;
}

/*
*
* get target's left and right boundary
*
*/
void FreeSpace::getHorizontalBound(Component& leftNeighbor, Component& rightNeighbor, Component* target) 
{

	int leftBound = INT_MIN, rightBound = INT_MAX;

	for (vector<Component*>::iterator it = comp.begin(); it != comp.end(); ++it) {
		// donnot compare to itself
		if (*it == target) continue;

		if (ifVerticalOverlap(target, *it)) {
			if ((*it)->get_ur_x() <= target->get_ll_x() && (*it)->get_ur_x() > leftBound) {
				leftNeighbor = *(*it);
				leftBound = (*it)->get_ur_x();
			}
			if (target->get_ur_x() <= (*it)->get_ll_x() && (*it)->get_ll_x() < rightBound) {
				rightNeighbor = *(*it);
				rightBound = (*it)->get_ll_x();
			}
		}
	}
	/* if no macro on target's left*/
	if (leftBound == INT_MIN) {
		for (vector<Bound*>::iterator it = bound.begin(); it != bound.end(); ++it) {
			Component *boundaryToComp = new Component(*it);
			if (ifVerticalOverlap(target, boundaryToComp) && boundaryToComp->get_ll_x() <= target->get_ll_x()
			&& boundaryToComp->get_ll_x() > leftBound) {
				leftNeighbor = *boundaryToComp;
				leftBound = boundaryToComp->get_ll_x();
			}
		}
	}
	/* if no macro on target's right*/
	if (rightBound == INT_MAX) {
		for (vector<Bound*>::iterator it = bound.begin(); it != bound.end(); ++it) {
			Component *boundaryToComp = new Component(*it);
			if (ifVerticalOverlap(target, boundaryToComp) && boundaryToComp->get_ll_x() >= target->get_ur_x()
			&& boundaryToComp->get_ll_x() < rightBound) {
				rightNeighbor = *boundaryToComp;
				rightBound = boundaryToComp->get_ll_x();
			}
		}
	}
}

void FreeSpace::getVerticalBound(Component& downNeighbor, Component& upNeighbor, Component* target) 
{

	int downBound = INT_MIN, upBound = INT_MAX;

	for (vector<Component*>::iterator it = comp.begin(); it != comp.end(); ++it) {
		// donnot compare to itself
		if (*it == target) continue;

		if (ifHorizontalOverlap(target, *it)) {
			if ((*it)->get_ur_y() <= target->get_ll_y() && (*it)->get_ur_y() > downBound) {
				downNeighbor = *(*it);
				downBound = (*it)->get_ur_y();
			}
			if (target->get_ur_y() <= (*it)->get_ll_y() && (*it)->get_ll_y() < upBound) {
				upNeighbor = *(*it);
				upBound = (*it)->get_ll_y();
			}
		}
	}
	/* if no macro on target's left*/
	if (downBound == INT_MIN) {
		for (vector<Bound*>::iterator it = bound.begin(); it != bound.end(); ++it) {
			Component *boundaryToComp = new Component(*it);
			if (ifHorizontalOverlap(target, boundaryToComp) && boundaryToComp->get_ll_y() <= target->get_ll_y()
			&& boundaryToComp->get_ll_y() > downBound) {
				downNeighbor = *boundaryToComp;
				downBound = boundaryToComp->get_ll_y();
			}
		}
	}
	/* if no macro on target's right*/
	if (upBound == INT_MAX) {
		for (vector<Bound*>::iterator it = bound.begin(); it != bound.end(); ++it) {
			Component *boundaryToComp = new Component(*it);
			if (ifHorizontalOverlap(target, boundaryToComp) && boundaryToComp->get_ll_y() >= target->get_ur_y()
			&& boundaryToComp->get_ll_y() < upBound) {
				upNeighbor = *boundaryToComp;
				upBound = boundaryToComp->get_ll_y();
			}
		}
	}
}


void 
FreeSpace::optimizeHorizontalFreeSpace(Node* leftInterval, Node* rightInterval, Component* target)
{
	// do nothing to fix macro, boundary and not_free_space
	if (target->getType() == FIXED)
		return;
	
	int pwc = parser._pwc * parser.dbuPerMicron;
	int minSpace = parser._mcsbmc * parser.dbuPerMicron;
	int leftWidth = leftInterval->high - leftInterval->low;
	int rightWidth = rightInterval->high - rightInterval->low;

	/* both side are dead space*/
	if (leftWidth < pwc && rightWidth < pwc)
		return;
	/* both side are free space*/
	if (leftWidth >= pwc && rightWidth >= pwc)
		return;


	/* find target's closest neighbor*/
	Component leftBoundComp, rightBoundComp;

	getHorizontalBound(leftBoundComp, rightBoundComp, target);
	int leftBound = leftBoundComp.get_ur_x();
	int rightBound = rightBoundComp.get_ll_x();

	if (leftWidth < pwc) {
		int movement = pwc - leftWidth;
		int overlapLength = std::max(0, min(target->get_ur_y(), leftBoundComp.get_ur_y()) - max(target->get_ll_y(), leftBoundComp.get_ll_y()));
		int moveCost = abs(parser._wa * movement);
		int areaCost = abs(parser._wb * overlapLength * leftWidth);
		if (target->get_ur_x() + movement <= rightBound - pwc &&
			areaCost > moveCost) {
			target->_llx += movement;
			leftInterval->high += movement;
			rightInterval->low += movement;
		}
	}
	else if (rightWidth < pwc) {
		int movement = -(pwc - rightWidth);
		int overlapLength = std::max(0, min(target->get_ur_y(), rightBoundComp.get_ur_y()) - max(target->get_ll_y(), rightBoundComp.get_ll_y()));
		int moveCost = abs(parser._wa * movement);
		int areaCost = abs(parser._wb * overlapLength * rightWidth);
		if (target->get_ll_x() + movement >= leftBound + pwc &&
			areaCost > moveCost) {
			target->_llx += movement;
			leftInterval->high += movement;
			rightInterval->low += movement;
		}
	}
}

void 
FreeSpace::optimizeVerticalFreeSpace(Node* downInterval, Node* upInterval, Component* target)
{
	
	if (target->isBoundary) // do nothing to boundary
		return;
	
	int pwc = parser._pwc * parser.dbuPerMicron;
	int minSpace = parser._mcsbmc * parser.dbuPerMicron;
	int downHeight = downInterval->high - downInterval->low;
	int upHeight = upInterval->high - upInterval->low;

	/* both side are dead space*/
	if (downHeight < pwc && upHeight < pwc)
		return;
	/* both side are free space*/
	if (downHeight >= pwc && upHeight >= pwc)
		return;


	/* find target's closest neighbor*/
	Component downBoundComp, upBoundComp;

	getVerticalBound(downBoundComp, upBoundComp, target);
	int downBound = downBoundComp.get_ur_y();
	int upBound = upBoundComp.get_ll_y();

	if (downHeight < pwc) {
		int movement = pwc - downHeight;
		int overlapLength = std::max(0, min(target->get_ur_x(), downBoundComp.get_ur_x()) - max(target->get_ll_x(), downBoundComp.get_ll_x()));
		int moveCost = abs(parser._wa * movement);
		int areaCost = abs(parser._wb * overlapLength * downHeight);
		if (target->get_ur_y() + movement <= upBound - pwc &&
			areaCost > moveCost) {
			target->_lly += movement;
			downInterval->high += movement;
			upInterval->low += movement;
		}
	}
	else if (upHeight < pwc) {
		int movement = -(pwc - upHeight);
		int overlapLength = std::max(0, min(target->get_ur_x(), upBoundComp.get_ur_x()) - max(target->get_ll_x(), upBoundComp.get_ll_x()));
		int moveCost = abs(parser._wa * movement);
		int areaCost = abs(parser._wb * overlapLength * upHeight);
		if (target->get_ll_y() + movement >= downBound + pwc &&
			areaCost > moveCost) {
			target->_lly += movement;
			downInterval->high += movement;
			upInterval->low += movement;
		}
	}
}

void
FreeSpace::vertical_freeSpace()
{
	priority_queue<Event> event;
	priority_queue<Event> boundary;
	IntervalTree line;

	for (vector<Bound*>::iterator it = bound.begin(); it != bound.end(); ++it) {
		bool begin;
		
		if ((*it)->getRelation() == DOWN) 
			begin = false;
		else if ((*it)->getRelation() == UP) 
			begin = true;
		else 
			continue;
		
		Component *boundaryToComp = new Component(*it);
		Event boundaryEvent(boundaryToComp, begin, boundaryToComp->get_ll_y(), true);
		boundaryEvent.bound = *it;
		boundary.push(boundaryEvent);
		//event.push(Event(*i));
	}

	for (unordered_set<Component*>::iterator i = not_free_space.begin(); i != not_free_space.end(); i++) {
		event.push(Event(*i, true, (*i)->get_ll_y()));
		event.push(Event(*i, false, (*i)->get_ur_y()));
	}

	for (vector<Component*>::iterator i = comp.begin(); i != comp.end(); i++) {
		if (std::find(_deletedComp.begin(), _deletedComp.end(), *i) != _deletedComp.end())
			continue;
		event.push(Event(*i, true, (*i)->get_ll_y()));
		event.push(Event(*i, false, (*i)->get_ur_y()));
	}

	while (!boundary.empty()) {

		Node* node;
		Event currentEvent;

		/* if only boundary remaining */
		if (event.empty()) {
			node = new Node(boundary.top().comp, V);
			currentEvent = boundary.top();
			boundary.pop();
			//cout << "boundary\n";
		}
		/* if only macro remaining */
		else if (boundary.empty()) {
			// won't happen
		}
		/* load low boundary */
		/* load high boundary */
		else if ((boundary.top().pos <= event.top().pos && boundary.top().comp->_boundary->getRelation() == DOWN) 
		|| (boundary.top().pos < event.top().pos && boundary.top().comp->_boundary->getRelation() == UP)) {
			node = new Node(boundary.top().comp, V);
			currentEvent = boundary.top();
			boundary.pop();
			//cout << "boundary\n";
		}
		/* load macro */
		else {
			node = new Node(event.top().comp, V);
			currentEvent = event.top();
			event.pop();
			//cout << "component\n";
		}

		if (currentEvent.begin == true) {
			//cout << "begin\n";
			Node* target = line.get_interval(line.root, node, V);

			/*if (target != NULL) 
				cout << "V target is NULL\n";*/

			int width = target->high - target->low;
			int height = currentEvent.pos - target->begin;

			//cout << "target: " << width << " " << height << endl;
			insertFreeSpace(target->low, target->begin, width, height, V);

			Node *left_interval = NULL;
			Node *right_interval = NULL;

			if (node->getLow() != target->getLow()) {
				left_interval = new Node(target->low, node->low);
				left_interval->begin = currentEvent.pos;
				left_interval->_leftComp = target->_leftComp;
				left_interval->_rightComp = node->comp;
				//cout << "left_interval: " << target->low << " ~ " << node->low << "  " << left_interval->begin << endl;
				DEBUG("left_interval\n");
			}
			if (node->getHigh() != target->getHigh()) {
				right_interval = new Node(node->high, target->high);
				right_interval->begin = currentEvent.pos;
				right_interval->_leftComp = node->comp;
				right_interval->_rightComp = target->_rightComp;
				DEBUG("rightt_interval\n");
				//cout << "right_interval: " << node->high << " ~ " << target->high << "  " << right_interval->begin << endl;
			}

			line.root = line.deleteNode(line.root, target);

			if (opt && left_interval != NULL && right_interval != NULL) {
				optimizeHorizontalFreeSpace(left_interval, right_interval, node->comp);
			}


			DEBUG("first\n");
			if (left_interval != NULL) {
				line.root = line.insert(line.root, left_interval);
				//cout << "insert left_interval: " << left_interval->_leftComp->name << " " << left_interval->_rightComp->name << endl;
			}
			//line.print_infix(line.root);

			DEBUG("second\n");
			if (right_interval != NULL) {
				line.root = line.insert(line.root, right_interval);
				//cout << "insert right_interval: " << right_interval->_leftComp->name << " " << right_interval->_rightComp->name << endl;
			}
			//line.print_infix(line.root);
		}
		else {
			//cout << "end\n";
			/*if (!currentEvent.isBound)
				DEBUG("\n\ndelete: " << node->comp->name << " " << node->low << " " << node->high << endl);*/

			Node* merge = new Node();

			DEBUG("delete left\n");
			Node *left_neighbor = line.find_left_adjacent_interval(line.root, node->low);
			if (left_neighbor != NULL) {
				// merge left left freespace
				merge->low = left_neighbor->low;
				merge->_leftComp = left_neighbor->_leftComp;
				DEBUG("left_neighbor: " << left_neighbor->low << " " << left_neighbor->high << endl);

				// add left freespace
				int width = left_neighbor->high - left_neighbor->low;
				int height = currentEvent.pos - left_neighbor->begin;

				insertFreeSpace(left_neighbor->low, left_neighbor->begin, width, height, V);

				line.root = line.deleteNode(line.root, left_neighbor);
				DEBUG("delete left\n");
			}
			else {
				merge->low = node->low;
				merge->_leftComp = node->comp;
			}

			//line.print_infix(line.root);

			// add right freespace
			DEBUG("delete right\n");
			Node *right_neighbor = line.find_right_adjacent_interval(line.root, node->high);
			if (right_neighbor != NULL) {
				// merge left right freespace
				merge->high = right_neighbor->high;
				merge->_rightComp = right_neighbor->_rightComp;
				DEBUG("right_neighbor: " << right_neighbor->low << " " << right_neighbor->high << endl);

				int width = right_neighbor->high - right_neighbor->low;
				int height = currentEvent.pos - right_neighbor->begin;
				
				insertFreeSpace(right_neighbor->low, right_neighbor->begin, width, height, V);

				line.root = line.deleteNode(line.root, right_neighbor);
				DEBUG("delete right\n");
			}
			else {
				merge->high = node->high;
				merge->_rightComp = node->comp;
			}


			//line.print_infix(line.root);
			merge->pos = (merge->low + merge->high) / 2.0;
			merge->max = merge->high;
			merge->begin = currentEvent.pos;


			DEBUG("insert: " << merge->low << " " << merge->high << endl);
			DEBUG("insert merge free space: " << merge->begin << endl);

			line.root = line.insert(line.root, merge);
			//cout << "merge: " << merge->_leftComp->name << " " << merge->_rightComp->name << endl;
			//line.print_infix(line.root);
		}
	}
}

void
FreeSpace::horizon_freeSpace()
{
	priority_queue<Event> event;
	priority_queue<Event> boundary;
	IntervalTree line;

	for (vector<Bound*>::iterator it = bound.begin(); it != bound.end(); it++) {
		bool begin;
		int relation = (*it)->getRelation();
		
		if (relation == LEFT) 
			begin = false;
		else if (relation == RIGHT) 
			begin = true;
		else continue;
		
		Component *boundaryToComp = new Component(*it);
		Event boundaryEvent(boundaryToComp, begin, boundaryToComp->get_ll_x(), true);
		boundaryEvent.bound = *it;
		boundary.push(boundaryEvent);
		//event.push(Event(*i));
	}

	for (unordered_set<Component*>::iterator i = not_free_space.begin(); i != not_free_space.end(); i++) {
		event.push(Event(*i, true, (*i)->get_ll_x()));
		event.push(Event(*i, false, (*i)->get_ur_x()));
	}

	for (vector<Component*>::iterator i = comp.begin(); i != comp.end(); i++) {
		if (std::find(_deletedComp.begin(), _deletedComp.end(), *i) != _deletedComp.end())
			continue;
		event.push(Event(*i, true, (*i)->get_ll_x()));
		event.push(Event(*i, false, (*i)->get_ur_x()));
	}

	while (!boundary.empty()) {

		Node* node;
		Event currentEvent;

		/* if only boundary remaining */
		if (event.empty()) {
			node = new Node(boundary.top().comp, H);
			currentEvent = boundary.top();
			boundary.pop();
			//cout << "boundary\n";
		}
		/* if only macro remaining */
		else if (boundary.empty()) {
			// won't happen
		}
		/* load low boundary */
		/* load high boundary */
		else if ((boundary.top().pos <= event.top().pos && boundary.top().comp->_boundary->getRelation() == LEFT) 
		|| (boundary.top().pos < event.top().pos && boundary.top().comp->_boundary->getRelation() == RIGHT)) {
			node = new Node(boundary.top().comp, H);
			currentEvent = boundary.top();
			boundary.pop();
			//cout << "boundary\n";
		}
		/* load macro */
		else {
			node = new Node(event.top().comp, H);
			currentEvent = event.top();
			event.pop();
			//cout << "component\n";
		}

		if (currentEvent.begin == true) {

			if (!currentEvent.isBound)
				DEBUG("\n\ninsert: " << node->comp->name << " " << node->low << " " << node->high << endl);

			Node* target = line.get_interval(line.root, node, H);

			//if (target == NULL) cout << "H target is NULL\n";

			int width = currentEvent.pos - target->begin;
			int height = target->high - target->low;

			insertFreeSpace(target->begin, target->low, width, height, H);

			Node *lower_interval = NULL;
			Node *upper_interval = NULL;

			if (target->low != node->low) {
				lower_interval = new Node(target->low, node->low);
				lower_interval->begin = currentEvent.pos;
				DEBUG("lower_interval\n");

			}
			if (node->high != target->high) {
				upper_interval = new Node(node->high, target->high);
				upper_interval->begin = currentEvent.pos;
				DEBUG("upper_interval\n");
			}

			line.root = line.deleteNode(line.root, target);

			if (opt && lower_interval != NULL && upper_interval != NULL) {
				optimizeVerticalFreeSpace(lower_interval, upper_interval, node->comp);
			}

			DEBUG("first\n");
			if (lower_interval != NULL) {
				line.root = line.insert(line.root, lower_interval);
			}
			//line.print_infix(line.root);


			DEBUG("second\n");
			if (upper_interval != NULL) {
				line.root = line.insert(line.root, upper_interval);
			}
			//line.print_infix(line.root);
		}
		else {
			if (!currentEvent.isBound)
				DEBUG("\n\ndelete: " << node->comp->name << " " << node->low << " " << node->high << endl);

			Node* merge = new Node();

			DEBUG("delete lower\n");
			Node *lower_neighbor = line.find_left_adjacent_interval(line.root, node->low);
			if (lower_neighbor != NULL) {
				DEBUG("delete lower\n");
				// merge left left freespace
				merge->low = lower_neighbor->low;
				DEBUG("lower_neighbor: " << lower_neighbor->low << " " << lower_neighbor->high << endl);

				// add lower freespace
				int width = currentEvent.pos - lower_neighbor->begin;
				int height = lower_neighbor->high - lower_neighbor->low;

				insertFreeSpace(lower_neighbor->begin, lower_neighbor->low, width, height, H);

				line.root = line.deleteNode(line.root, lower_neighbor);
				DEBUG("delete lower\n");
			}
			else
				merge->low = node->low;

			//line.print_infix(line.root);

			// add upper freespace
			DEBUG("delete upper\n");
			Node *upper_neighbor = line.find_right_adjacent_interval(line.root, node->high);
			if (upper_neighbor != NULL) {
				DEBUG("delete upper\n");
				// merge left right freespace
				merge->high = upper_neighbor->high;
				DEBUG("upper_neighbor: " << upper_neighbor->low << " " << upper_neighbor->high << endl);

				// add lower freespace
				int width = currentEvent.pos - upper_neighbor->begin;
				int height = upper_neighbor->high - upper_neighbor->low;
				
				insertFreeSpace(upper_neighbor->begin, upper_neighbor->low, width, height, H);

				line.root = line.deleteNode(line.root, upper_neighbor);
				DEBUG("delete upper\n");
			}
			else
				merge->high = node->high;

			//line.print_infix(line.root);

			merge->pos = (merge->low + merge->high) / 2.0;
			merge->setMax(merge->high);
			merge->begin = currentEvent.pos;

			line.root = line.insert(line.root, merge);
			//line.print_infix(line.root);
		}

	}
}

void 
FreeSpace::findFreeSpace()
{	
	free_space.clear();
	not_free_space.clear();
	opt = false;
	for (size_t i = 0; i < 2; ++i) {
		vertical_freeSpace();
		horizon_freeSpace();
		//opt = false;
	}
}