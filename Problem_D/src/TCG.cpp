#include "TCG.h"

Point* 
TCG::newPoint(Component* comp)
{
	Point* point = new Point(comp);
	_pnts[comp] = point;

	return point;
}

void 
TCG::buildTCG()
{
	/*************************************************************************
	*                                                                        *
	* Use Sweep line to build constraint grapg                               *
	* (Without transitive closure)                                           *
	*                                                                        *
	*************************************************************************/

	//----------------horizontal line from low to hight---------------------//
	//Initialize event
	priority_queue<Event> event;
	IntervalTree line;

	for (vector<Component*>::iterator i = _ogroup.begin(); i != _ogroup.end(); ++i) {
		event.push(Event(*i, true, (*i)->get_ll_y()));
		event.push(Event(*i, false, (*i)->get_ur_y()));
	}

	while (!event.empty())
	{
		Node* node = new Node(event.top().comp);

		if (event.top().begin == true) {

			// Determine the leftest block
			if (leftest == NULL ||
				node->getComp()->get_ll_x() < leftest->get_ll_x())
				leftest = node->getComp();
			// Determine the lowest block
			if (lowest == NULL
				|| node->getComp()->get_ll_y() < lowest->get_ll_y())
				lowest = node->getComp();
			// Determine the rightest block
			if (rightest == NULL
				|| node->getComp()->get_ur_x() > rightest->get_ur_x())
				rightest = node->getComp();
			// Determine the highest block
			if (highest == NULL
				|| node->getComp()->get_ur_y() > highest->get_ur_y())
				highest = node->getComp();

			vector<Component*> o_comp;
			line.search_overlap(o_comp, line.root, node, 0.0);

			// New a point
			Point *root = newPoint(node->getComp());
			
			for (vector<Component*>::iterator it = o_comp.begin(); it != o_comp.end(); ++it)
				insertPoint(root, _pnts[*it]);

			line.root = line.insert(line.root, node);
		}
		else {
			line.root = line.deleteNode(line.root, node);

			Point *p = _pnts[node->getComp()];

			double h_weight = p->getWidth() / 2.0;
			double v_weight = p->getHeight() / 2.0;

			if (p->left.empty()) {
				p->left.push_back(make_pair(_hSource, h_weight));
				_hSource->right.push_back(make_pair(p, h_weight));
			}
			if (p->right.empty()) {
				p->right.push_back(make_pair(_hSink, h_weight));
				_hSink->left.push_back(make_pair(p, h_weight));
			}
			if (p->down.empty()) {
				p->down.push_back(make_pair(_vSource, v_weight));
				_vSource->up.push_back(make_pair(p, v_weight));
			}
			if (p->up.empty()) {
				p->up.push_back(make_pair(_vSink, v_weight));
				_vSink->down.push_back(make_pair(p, v_weight));
			}
		}

		event.pop();
	}

	findNearBoundary();

	// Assign horizontal source's leftMost(rightMost) as the closest left boundary
	_hSource->_lm = leftest->get_ll_x();
	_hSource->_rm = _hSource->_lm;

	// Assign vertical source's downMost(upMost) as the closest down boundary
	_vSource->_dm = lowest->get_ll_y();
	_vSource->_um = _vSource->_dm;

	// calculate valid position 
	// (leftMost, rightMost)
	// (downMost, upMost)
	cal_validPos();
}

void 
TCG::insertPoint(Point *root, Point *add)
{
	double x_overlap = (root->getWidth() + add->getWidth()) / 2.0 - fabs(root->get_cx() - add->get_cx());
	double y_overlap = (root->getHeight() + add->getHeight()) / 2.0 - fabs(root->get_cy() - add->get_cy());

	bool relation = (x_overlap <= y_overlap) ? H : V;

	if (relation == H) {

		double right = fabs(add->getComp()->get_ll_x() - root->getComp()->get_ur_x());
		double left = fabs(add->getComp()->get_ur_x() - root->getComp()->get_ll_x());

		double weight = ((root->getWidth() + add->getWidth()) / 2.0) + _minSpace;

		// add is right of the root
		if (right < left) {
			root->right.push_back(make_pair(add, weight));
			add->left.push_back(make_pair(root, weight));
		}
		// add is left of the root
		else {
			add->right.push_back(make_pair(root, weight));
			root->left.push_back(make_pair(add, weight));
		}
	}
	else if (relation == V) {

		double up = fabs(add->getComp()->get_ll_y() - root->getComp()->get_ur_y());
		double down = fabs(add->getComp()->get_ur_y() - root->getComp()->get_ll_y());

		double weight = ((root->getHeight() + add->getHeight()) / 2.0) + _minSpace;

		// add is above the root
		if (up < down) {
			root->up.push_back(make_pair(add, weight));
			add->down.push_back(make_pair(root, weight));
		}
		// add is below the root
		else {
			add->up.push_back(make_pair(root, weight));
			root->down.push_back(make_pair(add, weight));
		}
	}
}

void 
TCG::cal_leftMost(Point* root, double leftMost)
{
	if (root->getType() != FIXED && leftMost > root->_lm)
		root->setLeftMost(leftMost);

	for (vector<pair<Point*, double>>::iterator it = root->right.begin(); it != root->right.end(); ++it)
		cal_leftMost((*it).first, root->_lm + (*it).second);

	return;
}

void 
TCG::cal_rightMost(Point* root, double rightMost)
{
	if (root->getType() != FIXED && rightMost < root->_rm)
		root->setRightMost(rightMost);

	for (vector<pair<Point*, double>>::iterator it = root->left.begin(); it != root->left.end(); ++it)
		cal_rightMost((*it).first, root->_rm - (*it).second);

	return;
}

void 
TCG::cal_downMost(Point* root, double downMost)
{
	if (root->getType() != FIXED && downMost > root->_dm)
		root->setDownMost(downMost);

	for (vector<pair<Point*, double>>::iterator it = root->up.begin(); it != root->up.end(); ++it)
		cal_downMost((*it).first, root->_dm + (*it).second);

	return;
}

void 
TCG::cal_upMost(Point* root, double upMost)
{
	if (root->getType() != FIXED && upMost < root->_um)
		root->setUpMost(upMost);

	for (vector<pair<Point*, double>>::iterator it = root->down.begin(); it != root->down.end(); ++it)
		cal_upMost((*it).first, root->_um - (*it).second);

	return;
}

void
TCG::cal_validPos()
{
	/*************************************************************************
	*                                                                        *
	* Determin position x                                                    *
	*                                                                        *
	*************************************************************************/
	cal_leftMost(_hSource, _hSource->_lm);

	// rightMost of terminal in Horizontal graph is equal to leftMost
	/*if ((rightest->get_ur_x() - leftest->get_ll_x()) > _hSink->_lm)
		_hSink->_rm = rightest->get_ur_x();
	else */
		_hSink->_rm = _hSource->_lm + _hSink->_lm; // "_hSink->_lm" here is represented citical path legth

	if (_hSink->_rm > limit[RIGHT])
		_hSink->_rm = limit[RIGHT];

	_hSink->_lm = _hSink->_rm;

	cal_rightMost(_hSink, _hSink->_rm);


	/*************************************************************************
	*                                                                        *
	* Determin position y                                                    *
	*                                                                        *
	*************************************************************************/
	cal_downMost(_vSource, _vSource->_dm);

	// rightMost of terminal in Horizontal graph is equal to leftMost
	/*if ((highest->get_ur_y() - lowest->get_ll_y()) > _vSink->_dm)
		_vSink->_um = highest->get_ur_y();
	else */
		_vSink->_um = _vSource->_dm + _vSink->_dm;	// "v_terminal->_dm" here is represented citical path legth

	if (_vSink->_um > limit[UP])
		_vSink->_um = limit[UP];

	_vSink->_dm = _vSink->_um;

	cal_upMost(_vSink, _vSink->_um);
}

void
TCG::decideXPosition(Point* current, double updatedLeftMost)
{
	if (current->getType() != FIXED) {

		// Update leftmost value according to the previous decided x
		if (updatedLeftMost > current->_lm) {

			current->_lm = updatedLeftMost;
			
			if (current->_lm > current->_rm)
				DEBUG("Warning: leftMost is greater than rightMost\n");
		}

		//if (current->getComp()->get_mid_x() < current->_lm)
			current->getComp()->set_cx(current->_lm);
		//else if (current->getComp()->get_mid_x() > current->_rm)
			//current->getComp()->set_cx(current->_rm);
	}

	for (vector<pair<Point*, double>>::iterator next = current->right.begin(); next != current->right.end(); ++next) {
			decideXPosition((*next).first, current->getComp()->get_mid_x() + (*next).second);
	}
}

void
TCG::decideYPosition(Point* current, double updatedRightMost)
{
	if (current->getType() != FIXED) {

		if (updatedRightMost > current->_dm) {

			current->_dm = updatedRightMost;

			if (current->_dm > current->_um)
				DEBUG("Warning: downMost is greater than upMost\n");
		}

		//if (current->getComp()->get_mid_y() < current->_dm)
			current->getComp()->set_cy(current->_dm);
		//else if (current->getComp()->get_mid_y() > current->_um)
			//current->getComp()->set_cy(current->_um);
	}

	for (vector<pair<Point*, double>>::iterator next = current->up.begin(); next != current->up.end(); ++next) {
			decideYPosition((*next).first, current->getComp()->get_mid_y() + (*next).second);
	}
}

void 
TCG::findNearBoundary()
{
	for (vector<Bound*>::iterator it = bound.begin(); it != bound.end(); ++it) {

		Bound *_b = *it;

		switch (_b->getRelation())
		{
		case DOWN: {

			double width = rightest->get_ur_x() - leftest->get_ll_x();
			double midX = (rightest->get_ur_x() + leftest->get_ll_x()) / 2.0;

			if ((width + _b->getWidth()) / 2.0 > fabs(midX - _b->get_mid_pos().first) && // Check if overlap
				fabs(_b->getPos() - lowest->get_ll_y()) < fabs(limit[DOWN] - lowest->get_ll_y())) // Compare distance with lowest line 
				limit[DOWN] = _b->getPos();

			break;
		}
		case UP: {

			double width = rightest->get_ur_x() - leftest->get_ll_x();
			double midX = (rightest->get_ur_x() + leftest->get_ll_x()) / 2.0;

			if ((width + _b->getWidth()) / 2.0 > fabs(midX - _b->get_mid_pos().first) && // Check if overlap
				fabs(_b->getPos() - highest->get_ur_y()) < fabs(limit[UP] - highest->get_ur_y())) // Compare distance with highest line 
					limit[UP] = _b->getPos();
				
			break;
		}
		case LEFT: {

			double height = highest->get_ur_y() - lowest->get_ll_y();
			double midY = (highest->get_ur_y() + lowest->get_ll_y()) / 2.0;

			if ((height + _b->getHeight()) / 2.0 > fabs(midY - _b->get_mid_pos().second) && // Check if overlap
				fabs(_b->getPos() - leftest->get_ll_x()) < fabs(limit[LEFT] - leftest->get_ll_x())) // Compare distance with leftest line 
				limit[LEFT] = _b->getPos();

			break;
		}
		case RIGHT: {

			double height = highest->get_ur_y() - lowest->get_ll_y();
			double midY = (highest->get_ur_y() + lowest->get_ll_y()) / 2.0;

			if ((height + _b->getHeight()) / 2.0 > fabs(midY - _b->get_mid_pos().second) && // Check if overlap
				fabs(_b->getPos() - rightest->get_ur_x()) < fabs(limit[RIGHT] - rightest->get_ur_x())) // Compare distance with rightest line 
				limit[RIGHT] = _b->getPos();

			
			break;
		}
		default:
			DEBUG("Undefinded boundary's relation\n");
			break;
		}
		
	}
}

void 
TCG::optimizePosition()
{
	/*double dx;
	double deltaX = 0.0;

	// delta
	for (vector<Component*>::iterator it = _ogroup.begin(); it != _ogroup.end(); ++it) 
		deltaX += ((*it)->_originX - (*it)->get_mid_x());

	dx = deltaX / _ogroup.size();

	for (vector<Component*>::iterator it = _ogroup.begin(); it != _ogroup.end(); ++it)
		(*it)->x += dx;*/

	double dy;
	double deltaY = 0.0;

	// delta
	for (vector<Component*>::iterator it = _ogroup.begin(); it != _ogroup.end(); ++it)
		deltaY += ((*it)->_originY - (*it)->get_mid_y());

	dy = deltaY / _ogroup.size();

	for (vector<Component*>::iterator it = _ogroup.begin(); it != _ogroup.end(); ++it)
		(*it)->_cy += dy;
}

void 
TCG::printHTCG(Point* root)
{
	if (root == _hSource)
		cout << "h_sink" << ": " << '(' << root->_lm << ' ' << root->_rm << ')' << endl;
	else if (root == _hSink)
		cout << "h_terminal" << ": " << '(' << root->_lm << ' ' << root->_rm << ')' << endl;
	else {
		cout << root->getName() << ": " << '(' << root->_lm << ' ' << root->_rm << ')' << endl;
		//cout << root->getName() << "'s x: " << root->getComp()->get_mid_x() << endl;
	}

	for (vector<pair<Point*, double>>::iterator it = root->right.begin(); it != root->right.end(); ++it)
		printHTCG((*it).first);
}

void 
TCG::printVTCG(Point* root)
{
	if (root == _vSource)
		cout << "v_sink" << ": " << '(' << root->_dm << ' ' << root->_um << ')' << endl;
	else if (root == _vSink)
		cout << "v_terminal" << ": " << '(' << root->_dm << ' ' << root->_um << ')' << endl;
	else {
		cout << root->getName() << ": " << '(' << root->_dm << ' ' << root->_um << ')' << endl;
		//cout << root->getName() << "'s y: " << root->getComp()->get_mid_y() << endl;
	}

	for (vector<pair<Point*, double>>::iterator it = root->up.begin(); it != root->up.end(); ++it)
		printVTCG((*it).first);
}