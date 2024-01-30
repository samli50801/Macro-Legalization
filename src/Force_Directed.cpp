#include "Force_Directed.h"
#include <limits.h>

bool endFD = false;

void Force_Directed::addToGroup(vector<vector<Component *>> &_grps, Component *root, Component *added)
{
	if (root->isBoundary && added->isBoundary)
		return;
	if (root->type == FIXED && added->type == FIXED)
		return;

	vector<vector<Component *>>::iterator rt_grp = _grps.end();
	vector<vector<Component *>>::iterator add_grp = _grps.end();

	for (vector<vector<Component *>>::iterator _grp = _grps.begin(); _grp != _grps.end(); _grp++)
	{

		if (rt_grp == _grps.end())
			if (find(_grp->begin(), _grp->end(), root) != _grp->end())
				rt_grp = _grp;

		if (add_grp == _grps.end())
			if (find(_grp->begin(), _grp->end(), added) != _grp->end())
				add_grp = _grp;
	}

	if (rt_grp == _grps.end() && add_grp == _grps.end())
	{ // both are not in the group
		vector<Component *> temp;
		temp.push_back(root);
		temp.push_back(added);
		_grps.push_back(temp);
	}
	else if (rt_grp != _grps.end() && add_grp == _grps.end())
	{ // root is in the group
		rt_grp->push_back(added);
	}
	else if (rt_grp == _grps.end() && add_grp != _grps.end())
	{ // added is in the group
		add_grp->push_back(root);
	}
	else
	{ // both are already in the group
		if (rt_grp == add_grp)
			return; // they both in the same group

		if (rt_grp->size() > add_grp->size())
		{
			rt_grp->insert(rt_grp->end(), add_grp->begin(), add_grp->end());
			_grps.erase(add_grp);
		}
		else
		{
			add_grp->insert(add_grp->end(), rt_grp->begin(), rt_grp->end());
			_grps.erase(rt_grp);
		}
	}
}

void Force_Directed::genOverlapGroup()
{
	//---------------horizontal line from low to high---------------------//
	//Initialize event
	priority_queue<Event> event;
	IntervalTree line;
	line.setMinSpace(_minSpace);
	_grps.clear();

	for (vector<Bound *>::iterator it = bound.begin(); it != bound.end(); it++)
	{
		Component *c = new Component(*it);
		//c->expandWidth(_minSpace);	// Expand the boundary
		//c->expandHeight(_minSpace);	// Expand the boundary
		c->setForce(0.0, 0.0);

		Event b1(c, true, c->get_ll_y(), true);
		event.push(b1);
		Event b2(c, false, c->get_ur_y(), true);
		event.push(b2);
	}

	for (vector<Component *>::iterator i = comp.begin(); i != comp.end(); i++)
	{
		(*i)->expandWidth(_minSpace);  // Expand the boundary
		(*i)->expandHeight(_minSpace); // Expand the boundary

		(*i)->setForce(0.0, 0.0);
		event.push(Event(*i, true, (*i)->get_ll_y()));
		event.push(Event(*i, false, (*i)->get_ur_y()));
	}

	endFD = true;
	// Initialize design boundary width
	_dw.clear();

	while (!event.empty())
	{
		Node *node = new Node(event.top().comp);

		if (event.top().begin == true)
		{
			if (event.top().isBound)
				dw_handler(node->comp);

			vector<Component *> o_comp;

			line.search_overlap_for_overlap_group(o_comp, line.root, node);
			for (vector<Component *>::iterator it = o_comp.begin(); it != o_comp.end(); it++)
			{
				//cout << "node name: " << node->comp->getName() << endl;
				//cout << "overlap name: " << (*it)->getName() <<endl;
				force_calculation(node->getComp(), *it);
				endFD = false;
			}

			line.root = line.insert(line.root, node);
		}
		else
		{
			line.root = line.deleteNode(line.root, node);

			if (!event.top().isBound)
			{
				event.top().comp->shrinkWidth(_minSpace);  // Shrink width
				event.top().comp->shrinkHeight(_minSpace); // Shrink height
			}

			// Apply final force to macro
			if (node->getComp()->type != FIXED)
			{
				event.top().comp->_llx += event.top().comp->getForce().first;
				event.top().comp->_lly += event.top().comp->getForce().second;
				//	experiment
				//event.top().comp->_originX = event.top().comp->_llx;
				//event.top().comp->_originY = event.top().comp->_lly;
			}
		}
		event.pop();
	}
}

void Force_Directed::force_calculation(Component *a, Component *b)
{
	if (a->isBoundary && b->isBoundary)
		return;

	DEBUG(a->name << " origin: " << a->_force.first << "  " << a->_force.second << endl);
	DEBUG(b->name << " origin: " << b->_force.first << "  " << b->_force.second << endl);

	bool force_dir;

	if (!a->isBoundary && !b->isBoundary)
	{

		// Calculated force
		double _dis;

		double zero = 0.0;
		double ex = 0;

		double x1 = b->get_ur_x() - a->get_ll_x();
		if (x1 > 0)
			x1 += ex;
		else
			x1 -= ex;

		double x2 = b->get_ll_x() - a->get_ur_x();
		if (x2 > 0)
			x2 += ex;
		else
			x2 -= ex;

		double y1 = b->get_ur_y() - a->get_ll_y();
		if (y1 > 0)
			y1 += ex;
		else
			y1 -= ex;

		double y2 = b->get_ll_y() - a->get_ur_y();
		if (y2 > 0)
			y2 += ex;
		else
			y2 -= ex;

		double ol_x_dis = min(fabs(x1), fabs(x2));
		double ol_y_dis = min(fabs(y1), fabs(y2));

		double left = fMax(a->get_ll_x(), b->get_ll_x());
		double right = fMin(a->get_ur_x(), b->get_ur_x());
		double down = fMax(a->get_ll_y(), b->get_ll_y());
		double up = fMin(a->get_ur_y(), b->get_ur_y());

		double h_dis = max(zero, right - left);
		double v_dis = max(zero, up - down);

		double ol_area = h_dis * v_dis;

		double total_area = b->getArea() + a->getArea();

		double a_ratio;
		double b_ratio;

		if (a->type == FIXED && b->type == FIXED)
		{
			a_ratio = 0.0;
			b_ratio = 0.0;
		}
		else if (a->type == FIXED)
		{
			a_ratio = 1.0;
			b_ratio = 0.0;
		}
		else if (b->type == FIXED)
		{
			a_ratio = 0.0;
			b_ratio = 1.0;
		}
		else
		{
			a_ratio = a->getArea() / total_area;
			b_ratio = 1.0 - a_ratio;		
		}

		if (ol_x_dis <= ol_y_dis)
		{

			_dis = (fabs(x1) <= fabs(x2)) ? fabs(x1) : -fabs(x2);

			DEBUG("_dis: " << _dis << "b_ratio" << b_ratio << endl);
			DEBUG(a->name << " overlap x: " << a->_force.first << " => ");
			m_2_m_force(a, a->_force.first, b_ratio * _dis);
			DEBUG(a->_force.first << endl);

			DEBUG(b->name << " overlap x: " << b->_force.first << " => ");
			m_2_m_force(b, b->_force.first, -a_ratio * _dis);
			DEBUG(b->_force.first << endl);

			force_dir = H;

			// Keep macro inside design boundary
			// Use design width to determine whether it is inside the boundary(Only for horizontal movement)
			if (!isInsideBoundary(a) && a->type != FIXED)
				limitedInBoundary(a, force_dir);
			if (!isInsideBoundary(b) && b->type != FIXED)
				limitedInBoundary(b, force_dir);
		}
		else if (ol_y_dis < ol_x_dis)
		{

			_dis = (fabs(y1) <= fabs(y2)) ? fabs(y1) : -fabs(y2);

			DEBUG(a->name << " overlap y: " << a->_force.second << " => ");
			m_2_m_force(a, a->_force.second, b_ratio * _dis);
			DEBUG(a->_force.second << endl);

			DEBUG(b->name << " overlap y: " << b->_force.second << " => ");
			m_2_m_force(b, b->_force.second, -a_ratio * _dis);
			DEBUG(b->_force.second << endl);

			force_dir = V;

			// Keep macro inside design boundary
			// CANNOT Use design width to determine whether it is inside the boundary when it is vertical movement
			if (a->type != FIXED)
				limitedInBoundary(a, force_dir);
			if (b->type != FIXED)
				limitedInBoundary(b, force_dir);
		}
	}
	else
	{
		// Keep macro inside design boundary
		if (a->isBoundary)
		{
			force_dir = !a->_boundary->dir;
			limitedInBoundary(b, force_dir);
		}
		else
		{
			force_dir = !b->_boundary->dir;
			limitedInBoundary(a, force_dir);
		}
	}

	DEBUG(a->name << " final: " << a->_force.first << "  " << a->_force.second << endl);
	DEBUG(b->name << " final: " << b->_force.first << "  " << b->_force.second << endl);
}

void Force_Directed::do_forceDirected()
{
	int round = 0;
	
	while (!endFD)
	{
		cout << "Iteration: " << round << endl;
		// If iteration exceed the threashold, then end while
		if (round > 4000) //best 4000
			break;

		round++;

		DEBUG("\n-----------------------------classify into groups---------------------------------------\n\n");
		/*for (vector<vector<Component*> >::iterator it = _grps.begin(); it != _grps.end();)
			it = _grps.erase(it);*/

		genOverlapGroup();
	}
}

void Force_Directed::do_forceDirected_ver2()
{
	vector<int> wait_to_erase;

	for (size_t round = 0; round < 1; ++round)
	{

		DEBUG("\n-----------------------------classify into groups---------------------------------------\n\n");

		_grps.clear();

		generate_overlap_group();

		for (vector<vector<Component*> >::iterator it = _grps.begin(); it != _grps.end(); ++it)
		//for (map<string, vector<Component *>>::iterator it = group.begin(); it != group.end(); ++it) //group
		{

			TCG tcg;
			tcg.setMinSpace(_minSpace);
			tcg.set_ogroup((*it));
			tcg.buildTCG();

			//cout << "\nHorizontal CG: " << endl;
			//tcg.printHTCG(tcg.getHSource());
			//cout << "\nVertical CG: " << endl;
			//tcg.printVTCG(tcg.getVSource());

			tcg.decideXPosition(tcg.getHSource(), tcg.getHSource()->_lm);
			tcg.decideYPosition(tcg.getVSource(), tcg.getVSource()->_dm);
			tcg.optimizePosition();

			//cout << "\nActural Horizontal CG: " << endl;
			//tcg.printHTCG(tcg.getHSource());
			//cout << "\nActural Vertical CG: " << endl;
			//tcg.printVTCG(tcg.getVSource());

			//cout << "rightest: " << tcg.getRightestBlock()->getName() << endl;
			//cout << "highest: " << tcg.getHighestBlock()->getName() << endl;

			//for (auto j = (*it).begin(); j != (*it).end(); ++j) {
			//	cout << (*j)->getName() << "': " << (*j)->get_mid_x() << ", " << (*j)->get_mid_y() << endl;
			//}
		}
	}
}

void Force_Directed::generate_overlap_group()
{
	//---------------horizontal line from low to hight---------------------//
	//Initialize event
	priority_queue<Event> event;
	IntervalTree line;
	line.setMinSpace(_minSpace);
	/* Initialize overlap group */
	_grps.clear();

	/* put all boundaries into event */
	for (vector<Bound *>::iterator it = bound.begin(); it != bound.end(); it++)
	{
		Component *c = new Component(*it);
		// Do not expand boundary width and height like macro because it would influent dw_handler
		Event b1(c, true, c->get_ll_y(), true);
		event.push(b1);
		Event b2(c, false, c->get_ur_y(), true);
		event.push(b2);
	}
	/* put all macros into event */
	for (vector<Component *>::iterator i = comp.begin(); i != comp.end(); i++)
	{
		(*i)->expandWidth(_minSpace);  // Expand the boundary
		(*i)->expandHeight(_minSpace); // Expand the boundary

		event.push(Event(*i, true, (*i)->get_ll_y()));
		event.push(Event(*i, false, (*i)->get_ur_y()));
	}

	while (!event.empty())
	{
		Node *node = new Node(event.top().comp);

		if (event.top().begin == true)
		{
			vector<Component *> o_comp;

			line.search_overlap_for_overlap_group(o_comp, line.root, node);

			for (vector<Component *>::iterator it = o_comp.begin(); it != o_comp.end(); it++) {
				addToGroup(_grps, *it, node->getComp());
			}

			line.root = line.insert(line.root, node);
		}
		else
		{
			if (!event.top().isBound)
			{
				event.top().comp->shrinkWidth(_minSpace);  // Shrink width
				event.top().comp->shrinkHeight(_minSpace); // Shrink height
			}

			line.root = line.deleteNode(line.root, node);
		}
		event.pop();
	}
}

void Force_Directed::genOverlapGroup_ver2()
{
	//---------------horizontal line from low to hight---------------------//
	//Initialize event
	priority_queue<Event> event;
	IntervalTree line;

	for (vector<Bound *>::iterator it = bound.begin(); it != bound.end(); it++)
	{
		Component *c = new Component(*it);
		//c->expandWidth(_minSpace);	// Expand the boundary
		//c->expandHeight(_minSpace);	// Expand the boundary
		c->setForce(0.0, 0.0);

		Event b1(c, true, c->get_ll_y(), true);
		event.push(b1);
		Event b2(c, false, c->get_ur_y(), true);
		event.push(b2);
	}

	for (vector<Component *>::iterator i = comp.begin(); i != comp.end(); i++)
	{
		(*i)->expandWidth(_minSpace);  // Expand the boundary
		(*i)->expandHeight(_minSpace); // Expand the boundary

		(*i)->setForce(0.0, 0.0);
		event.push(Event(*i, true, (*i)->get_ll_y()));
		event.push(Event(*i, false, (*i)->get_ur_y()));
	}

	while (!event.empty())
	{
		Node *node = new Node(event.top().comp);

		if (event.top().begin == true)
		{

			if (event.top().isBound)
				dw_handler(node->comp);

			vector<Component *> o_comp;

			line.search_overlap(o_comp, line.root, node, _minSpace);

			for (vector<Component *>::iterator it = o_comp.begin(); it != o_comp.end(); it++)
			{
				node->comp->inGroup = true; // For drawing
				(*it)->inGroup = true;		// For drawing
				addToGroup(_grps, *it, node->getComp());
			}

			line.root = line.insert(line.root, node);
		}
		else
		{

			line.root = line.deleteNode(line.root, node);

			if (!event.top().isBound)
			{

				event.top().comp->shrinkWidth(_minSpace);  // Shrink width
				event.top().comp->shrinkHeight(_minSpace); // Shrink height
			}
		}
		event.pop();
	}
}

void Force_Directed::limitedInBoundary(Component *c, bool force_dir)
{
	// Mininum force direction(DOWN UP LEFT RIGHT or -1)
	int min = -1;
	// Mininum force
	double limit_force = DBL_MAX;

	for (vector<Bound *>::iterator boundary = bound.begin(); boundary != bound.end(); ++boundary)
	{

		// Only check the boundary direction we want
		if ((*boundary)->dir == force_dir)
			continue;

		Bound *_b = *boundary;

		// x or y's position after force added
		double new_pos;
		// Shrink width and height to the origin size
		double shrink_ht = c->getHeight() - _minSpace;
		double shrink_wd = c->getWidth() - _minSpace;

		switch (_b->getRelation())
		{
		case DOWN:
		{
			// new middle x
			double _mx = c->get_mid_x() + c->getForce().first;
			double shrink_ly = c->get_ll_y() + 0.5 * _minSpace;
			double o_x = (shrink_wd + _b->getWidth()) / 2.0 - fabs(_mx - _b->get_mid_pos().first);
			new_pos = shrink_ly + c->getForce().second;
			if (o_x > EPSILON && (new_pos < _b->getPos()))
			{
				m_2_b_force(limit_force, _b->getPos() - shrink_ly);
				min = DOWN;
				DEBUG(c->name << "'s y dforce=>" << limit_force << endl);
			}
			break;
		}
		case UP:
		{
			// new middle x
			double _mx = c->get_mid_x() + c->getForce().first;
			double shrink_uy = c->get_ur_y() - 0.5 * _minSpace;
			double o_x = (shrink_wd + _b->getWidth()) / 2.0 - fabs(_mx - _b->get_mid_pos().first);
			new_pos = shrink_uy + c->getForce().second;
			DEBUG("o_x: " << o_x << endl);
			if (o_x > EPSILON && (new_pos > _b->getPos()))
			{
				m_2_b_force(limit_force, _b->getPos() - shrink_uy);
				min = UP;
				DEBUG(c->name << "'s y uforce=>" << limit_force << endl);
			}
			break;
		}
		case LEFT:
		{
			// new middle y
			double _my = c->get_mid_y() + c->getForce().second;
			double shrink_lx = c->get_ll_x() + 0.5 * _minSpace;
			double o_y = (shrink_ht + _b->getHeight()) / 2.0 - fabs(_my - _b->get_mid_pos().second);
			new_pos = shrink_lx + c->getForce().first;
			if (o_y > EPSILON && (new_pos < _b->getPos()))
			{
				m_2_b_force(limit_force, _b->getPos() - shrink_lx);
				min = LEFT;
				DEBUG(c->name << "'s x lforce=>" << limit_force << endl);
			}
			break;
		}
		case RIGHT:
		{
			// new middle y
			double _my = c->get_mid_y() + c->getForce().second;
			double shrink_ux = c->get_ur_x() - 0.5 * _minSpace;
			double o_y = (shrink_ht + _b->getHeight()) / 2.0 - fabs(_my - _b->get_mid_pos().second);
			new_pos = shrink_ux + c->getForce().first;
			if (o_y > EPSILON && (new_pos > _b->getPos()))
			{
				m_2_b_force(limit_force, _b->getPos() - shrink_ux);
				min = RIGHT;
				DEBUG(c->name << "'s x rforce=>" << limit_force << endl);
			}
			break;
		}
		default:
			DEBUG("Undefinded boundary's relation\n");
			break;
		}
	}

	if (min == -1)
		return;
	else if (min == DOWN || min == UP)
		c->_force.second = limit_force;
	else if (min == LEFT || min == RIGHT)
		c->_force.first = limit_force;
}

void Force_Directed::m_2_m_force(Component *c, double &_origin, double _new)
{
	double dir = _origin * _new;

	if (_origin == 0.0 || _new == 0.0 ||
		(dir > 0.0 && fabs(_new) < fabs(_origin)))
		_origin = _new;
	// option 1
	else if (dir < 0.0 && fabs(_new) < fabs(_origin))
	{
		_origin = _new;
	}
}

void Force_Directed::m_2_b_force(double &_origin, double _new)
{
	if ((fabs(_new) < fabs(_origin)))
		_origin = _new;
}

void Force_Directed::dw_handler(Component *c)
{
	if (c->_boundary->dir != H)
		return;

	Bound *_b = c->_boundary;

	switch (_b->getRelation())
	{
	case DOWN:
	{

		if (_dw.empty())
			_dw.push_back(make_pair(_b->end, _b->start));
		else
		{
			int _lp = _b->end;
			int _rp = _b->start;
			vector<pair<int, int>>::iterator it;

			// Find target
			for (it = _dw.begin(); it != _dw.end(); ++it)
			{
				// Left expand _dw
				if (_rp == (*it).first)
				{
					(*it).first -= _b->getWidth();
					break;
				}
				// Left expand _dw
				else if (_lp == (*it).second)
				{
					(*it).second += _b->getWidth();
					break;
				}
			}

			if (it == _dw.end())
				DEBUG("error: _dw expand target haven't been found!");
		}

		break;
	}

	case UP:
	{

		int _lp = _b->start;
		int _rp = _b->end;
		vector<pair<int, int>>::iterator it;

		// Find target
		for (it = _dw.begin(); it != _dw.end(); ++it)
		{
			// Left shrink
			if (fEq(_lp, (*it).first))
			{
				(*it).first += _b->getWidth();
				break;
			}
			// Right shrink
			else if (_rp == (*it).second)
			{
				(*it).second -= _b->getWidth();
				break;
			}
			// Cut in the middle
			else if (_lp > (*it).first && _rp < (*it).first)
			{

				pair<int, int> _left = {(*it).first, _lp};
				pair<int, int> _right = {_rp, (*it).second};
				_dw.push_back(_left);
				_dw.push_back(_right);
				_dw.erase(it);
				it = _dw.end();
				break;
			}
		}

		if (it == _dw.end())
			DEBUG("error: _dw shrink target haven't been found!");

		break;
	}
	}
}

bool Force_Directed::isInsideBoundary(Component *c)
{
	double shrinkWid = c->getWidth() - _minSpace;
	double lowX = (c->get_mid_x() - 0.5 * shrinkWid) + c->getForce().first;
	double highX = (c->get_mid_x() + 0.5 * shrinkWid) + c->getForce().first;

	for (vector<pair<int, int>>::iterator seg = _dw.begin(); seg != _dw.end(); ++seg)
	{
		if (lowX >= seg->first && highX <= seg->second)
			return true;
	}
	return false;
}

void Force_Directed::move_macro_inside_boundary()
{
	bool isAllInBoundary = false;

	/* if macro can't be moved into the boundary after a specific iteration, then give this duty to TCG*/
	int threshold = 200;

	while (!isAllInBoundary && --threshold) {
		isAllInBoundary = true;
		_dw.clear();

		//---------------horizontal line from low to hight---------------------//
		// Initialize event
		priority_queue<Event> event;	// store macros lower and upper bound
		priority_queue<Event> boundary;	// store horizontal boundary

		/* put boundaries into event */
		for (vector<Bound *>::iterator it = bound.begin(); it != bound.end(); it++) {
			if((*it)->dir == H) {
				Component *boundaryToComp = new Component(*it);
				Event boundaryEvent(boundaryToComp, true, boundaryToComp->get_ll_y(), true);
				boundary.push(boundaryEvent);
			}
		}
		/* put all macro into event */
		for (vector<Component *>::iterator i = comp.begin(); i != comp.end(); i++) {
			event.push(Event(*i, true, (*i)->get_ll_y()));
			event.push(Event(*i, false, (*i)->get_ur_y()));
		}
		
		while (!event.empty() || !boundary.empty()) {
			Component* currentComp;

			/* if only boundary remaining */
			if (event.empty()) {
				currentComp = boundary.top().comp;
				boundary.pop();
			}
			/* if only macro remaining */
			else if (boundary.empty()) {
				currentComp = event.top().comp;
				event.pop();
			}
			/* load low boundary */
			/* load high boundary */
			else if ((boundary.top().pos <= event.top().pos && boundary.top().comp->_boundary->getRelation() == DOWN) 
			|| (boundary.top().pos < event.top().pos && boundary.top().comp->_boundary->getRelation() == UP)) {
				currentComp = boundary.top().comp;
				boundary.pop();
			}
			/* load macro */
			else {
				currentComp = event.top().comp;
				event.pop();
			}

			if (currentComp->isBoundary) 
				dw_handler(currentComp);
			else 
				macro_boundary_checker(currentComp, isAllInBoundary);
		}
	}
}

void Force_Directed::macro_boundary_checker(Component *c, bool &isAllInBoundary)
{
	bool legal = false;

	for (vector<pair<int, int>>::iterator seg = _dw.begin(); seg != _dw.end(); ++seg) {
		if (c->get_ll_x() >= seg->first && c->get_ur_x() <= seg->second) {
			legal = true;
			break;
		}
	}

	if (legal)
		return;
	else
	{
		isAllInBoundary = false;

		pair<int, int> min = { INT_MAX, 0};

		for (vector<Bound *>::iterator it = bound.begin(); it != bound.end(); ++it)
		{
			Bound *_b = *it;
			int min_distance = abs(min.first + min.second);

			switch (_b->getRelation())
			{
			case DOWN:
			{
				int move_distance = abs(_b->getPos() - c->get_ll_y());
				if (c->get_ll_y() < _b->getPos() && // if it is out of boundary
					move_distance < min_distance)	// choose the minimum distance
				{
					min.first = 0.0;
					min.second = +move_distance;
				} // Compare distance with lowest line

				break;
			}
			case UP:
			{
				int move_distance = abs(_b->getPos() - c->get_ur_y());
				if (c->get_ur_y() > _b->getPos() && // if it is out of boundary
					move_distance < min_distance)	// choose the minimum distance
				{
					min.first = 0.0;
					min.second = -move_distance;
				} // Compare distance with highest line

				break;
			}
			case LEFT:
			{
				int move_distance = abs(_b->getPos() - c->get_ll_x());
				if (c->get_ll_x() < _b->getPos() && // if it is out of boundary
					move_distance < min_distance)	// choose the minimum distance
				{
					min.first = +move_distance;
					min.second = 0.0;
				} // Compare distance with leftest line

				break;
			}
			case RIGHT:
			{

				int move_distance = abs(_b->getPos() - c->get_ur_x());
				if (c->get_ur_x() > _b->getPos() && // if it is out of boundary
					move_distance < min_distance)	// choose the minimum distance
				{
					min.first = -move_distance;
					min.second = 0.0;
				} // Compare distance with rightest line

				break;
			}
			default:
				DEBUG("Undefinded boundary's relation\n");
				break;
			}
		}
		cout << "min: " << min.first << " " << min.second << endl;
		if (c->getType() != FIXED) {
			c->_llx += min.first;
			c->_lly += min.second;
			c->_originX = c->_llx;
			c->_originY = c->_lly;
		}
	}
}