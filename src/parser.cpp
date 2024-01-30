#include "parser.h"

void 
Parser::readDef(string def_file, fstream& output, map<string, Macro*> &macro, vector<Component*> &comp)
{
	fstream _def(def_file, ios::in);

	if (!_def)
		DEBUG("Can't open file!\n");

	string buffer;
	_def >> buffer >> version >> buffer;
	output << "VERSION " << version << " ;\n";
	_def >> buffer >> designName >> buffer;
	output << "DESIGN " << designName << " ;\n";
	_def >> buffer >> buffer >> buffer >> dbuPerMicron >> buffer;
	output << "UNITS DISTANCE MICRONS " << dbuPerMicron << " ;\n\n";

	output << "DIEAREA ";
	while (_def >> buffer) {

		if (buffer == ";") break;
		
		if (buffer == "(") {
			int x, y;
			_def >> x >> y;

			/* set parameter for normalization */
			if (x < minOfWidth)
				minOfWidth = x;
			if (y < minOfHeight)
				minOfHeight = y;
			if (x > maxOfWidth)
				maxOfWidth = x;
			if (y > maxOfHeight)
				maxOfHeight = y;

			dieArea.push_back(make_pair(x, y));

			output << "( " << x << ' ' << y << " ) ";
		}
	}
	output << ";\n\n";
	
	_def >> buffer >> numComps >> buffer;
	output << "COMPONENTS " << numComps << " ;\n";

	while (_def >> buffer) {

		if (buffer == "DESIGN") break;

		if (buffer == "-") {
			Component *temp = new Component;
			_def >> temp->name;

			std::istringstream ss(temp->name);
			std::string token;
			while(std::getline(ss, token, '/')) {
				//don't insert the last token which is the actual name of component
				if (ss.rdbuf()->in_avail() != 0)
					temp->nameToken.insert(token);
			}
			if (temp->nameToken.size() > longestTokenLength)
				longestTokenLength = temp->nameToken.size();

			_def >> buffer; // macro's name

			temp->macro = macro[buffer];

			temp->width = temp->macro->width * dbuPerMicron;
			temp->height = temp->macro->height * dbuPerMicron;

			_def >> buffer >> buffer; // + FIXED/PLACED
			if (buffer == "PLACED") temp->type = PLACED; else
			if (buffer == "FIXED") temp->type = FIXED;

			_def >> buffer;
			_def >> temp->_llx >> temp->_lly;

			temp->_cx = temp->_llx + 0.5 * temp->width;
			temp->_cy = temp->_lly + 0.5 * temp->height;
			// Used for sceore evaluation
			temp->_originX = temp->_llx;
			temp->_originY = temp->_lly;

			comp.push_back(temp);
		}
	}
}

void 
Parser::readLef(string lef_file, map<string, Macro*> &macro)
{
	fstream _lef(lef_file, ios::in);

	if (!_lef)
		DEBUG("Can't open file!\n");

	string buffer;
	while (_lef >> buffer) {

		if (buffer == "LIBRARY") break;

		if (buffer == "MACRO") {
			Macro *temp = new Macro;
			_lef >> temp->name;
			_lef >> buffer >> temp->width >> buffer >> temp->height;
			macro[temp->name] = temp;
		}
	}
}

void 
Parser::writeDef(fstream& output, vector<Component*>& comp)
{
	for (vector<Component*>::iterator it = comp.begin(); it != comp.end(); it++) {
		output << "   - " << (*it)->name << ' ' << (*it)->macro->name << '\n';

		if ((*it)->type == PLACED)	output << "      + PLACED"; else
		if ((*it)->type == FIXED)	output << "      + FIXED";

		output << " ( " << (*it)->get_ll_x() << ' ' << (*it)->get_ll_y() << " ) " << "N ;\n";
	}
	output << "END COMPONENTS\n\n\n";
	output << "END DESIGN";
}

void
Parser::readTxt(string txt_file)
{
	fstream _txt(txt_file, ios::in);

	if (!_txt)
		DEBUG("Can't open TXT file!\n");

	string buffer;
	while (_txt >> buffer) {
		if (buffer == "powerplan_width_constraint") {
			_txt >> _pwc;
		}
		else if (buffer == "minimum_channel_spacing_between_macros_constraint") {
			_txt >> _mcsbmc;
		}
		else if (buffer == "buffer_area_reservation_extended_distance_constraint") {
			_txt >> _baredc;
		}
		else if (buffer == "weight_alpha") {
			_txt >> _wa;
		}
		else if (buffer == "weight_beta") {
			_txt >> _wb;
		}
		else {
			DEBUG("bad keyword: " << buffer << endl);
		}
	}
}

void
Parser::load_bounding()
{
	size_t boundaryPointNum = dieArea.size();

	// the diearea is an rectangle by specifying two points
	if (boundaryPointNum == 2) {
		// find left, right, top, and down boundary point
		int leftX = dieArea[0].first, rightX = dieArea[1].first, topY = dieArea[0].second, downY = dieArea[1].second;

		if (leftX > rightX) {
			// swap
			int temp = leftX;
			leftX = rightX;
			rightX = temp;
		}

		if (downY > topY) {
			// swap
			int temp = downY;
			downY = topY;
			topY = temp;
		}

		// construct Bound object for left, right, top, down boundary
		Bound *left = new Bound(leftX, downY, leftX, topY);
		bound.push_back(left);
		Bound *top = new Bound(leftX, topY, rightX, topY);
		bound.push_back(top);
		Bound *right = new Bound(rightX, topY, rightX, downY);
		bound.push_back(right);
		Bound *down = new Bound(rightX, downY, leftX, downY);
		bound.push_back(down);
	}
	// the diearea is an polygon by a series of points
	else {
		// find the leftmost point, breaking ties by selecting upper point
		size_t leftmostPointIndex = 0;
		for (size_t i = 1; i < boundaryPointNum; ++i) {
			// current point is to the left of leftmostPoint
			if (dieArea[leftmostPointIndex].first > dieArea[i].first) {
				leftmostPointIndex = i;
			}
			// current point is above leftmostPoint with same x-coordinate
			else if (dieArea[leftmostPointIndex].first == dieArea[i].first
				&& dieArea[leftmostPointIndex].second < dieArea[i].second) {
				leftmostPointIndex = i;
			}
		}

		// get the index of next point of leftmostPoint
		size_t nextPointIndex = leftmostPointIndex + 1;
		if (nextPointIndex >= boundaryPointNum)
			nextPointIndex = 0;

		// determine clockwise or counter-clockwise order of DIEAREA statement
		bool isClockwise = false;
		if (dieArea[leftmostPointIndex].first < dieArea[nextPointIndex].first)
			isClockwise = true;

		if (isClockwise)
			DEBUG("DIEAREA statement is clockwise order.\n");
		else
			DEBUG("DIEAREA statement is counter-clockwise order.\n");

		// normally construct polygon boundary
		if (isClockwise) {
			for (int i = 0; i < boundaryPointNum - 1; ++i) {
				Bound *b = new Bound(dieArea[i].first, dieArea[i].second, dieArea[i + 1].first, dieArea[i + 1].second);
				bound.push_back(b);
			}
			// last point connects first point
			Bound *b = new Bound(dieArea[boundaryPointNum - 1].first, dieArea[boundaryPointNum - 1].second, dieArea[0].first, dieArea[0].second);
			bound.push_back(b);
		}
		// inversely construct polygon boundary
		else {
			for (int i = boundaryPointNum - 1; i > 0; --i) {
				Bound *b = new Bound(dieArea[i].first, dieArea[i].second, dieArea[i - 1].first, dieArea[i - 1].second);
				bound.push_back(b);
			}
			// first point connects last point
			Bound *b = new Bound(dieArea[0].first, dieArea[0].second, dieArea[boundaryPointNum - 1].first, dieArea[boundaryPointNum - 1].second);
			bound.push_back(b);
		}
	}
}


void
MCluster::set_cx(double cx)
{
	double dx = cx - _cx;
	_cx = cx;

	for (vector<Component*>::iterator it = _comps.begin(); it != _comps.end(); it++)
		(*it)->set_cx((*it)->get_mid_x() + dx);
}

void
MCluster::set_cy(double cy)
{
	double dy = cy - _cy;
	_cy = cy;

	for (vector<Component*>::iterator it = _comps.begin(); it != _comps.end(); it++)
		(*it)->set_cy((*it)->get_mid_y() + dy);
}

// Compare function
bool LowXComp(const Component* a, const Component* b) { return a->get_ll_x() < b->get_ll_x(); } // Compare Cluster bounding box
bool LowYComp(const Component* a, const Component* b) { return a->get_ll_y() < b->get_ll_y(); }	// Compare Cluster bounding box
bool HighXComp(const Component* a, const Component* b) { return a->get_ur_x() < b->get_ur_x(); } // Compare Cluster bounding box
bool HighYComp(const Component* a, const Component* b) { return a->get_ur_y() < b->get_ur_y(); } // Compare Cluster bounding box

void
MCluster::buildBoundingBox()
{
	vector<Component*>::iterator _minX = min_element(_comps.begin(), _comps.end(), LowXComp);
	vector<Component*>::iterator _minY = min_element(_comps.begin(), _comps.end(), LowYComp);
	vector<Component*>::iterator _maxX = max_element(_comps.begin(), _comps.end(), HighXComp);
	vector<Component*>::iterator _maxY = max_element(_comps.begin(), _comps.end(), HighYComp);

	_cx = ((*_maxX)->get_ur_x() + (*_minX)->get_ll_x()) / 2;
	_cy = ((*_maxY)->get_ur_y() + (*_minY)->get_ll_y()) / 2;
	_width = (*_maxX)->get_ur_x() - (*_minX)->get_ll_x();
	_height = (*_maxY)->get_ur_y() - (*_minY)->get_ll_y();
}

void 
Parser::score_evaluation(vector<Component*>& comp, unordered_set<Component*>& nfs)
{
	double _totalDis = 0.0;
	double _totalNFS = 0.0;

	for (vector<Component*>::iterator it = comp.begin(); it != comp.end(); it++) {
		double _dx = fabs((*it)->_llx - (*it)->_originX) / (double)dbuPerMicron;
		double _dy = fabs((*it)->_lly - (*it)->_originY) / (double)dbuPerMicron;
		_totalDis += _dx;
		_totalDis += _dy;
	}

	for (unordered_set<Component*>::iterator it = nfs.begin(); it != nfs.end(); it++) {
		double width = (double)(*it)->getWidth() / (double)dbuPerMicron;
		double height = (double)(*it)->getHeight() / (double)dbuPerMicron;
		_totalNFS += width * height;
	}
	
	cout << "Total Displacement: " << _totalDis << endl;
	cout << "Total Non_Free_Space: " << _totalNFS << endl;
	cout << "Target value: " << _wa * _totalDis + _wb * sqrt(_totalNFS) << endl;
}


/*
*
* the variables of linear programming must be non-negative
*
*/
void Parser::moveCoordinateSystem(vector<Component*>& compVec)
{
	/* the most lower-left coordinate */
	int leftMost = 0;
	int downMost = 0;

	/* find leftmost and downmost point through dieArea and macros */
	for (size_t i = 0; i < dieArea.size(); ++i) {
		if (dieArea[i].first < leftMost)
			leftMost = dieArea[i].first;
		if (dieArea[i].second < downMost)
			downMost = dieArea[i].second;
	}
	for (size_t i = 0; i < compVec.size(); ++i) {
		if (compVec[i]->get_ll_x() < leftMost)
			leftMost = compVec[i]->get_ll_x();
		if (compVec[i]->get_ll_y() < downMost)
			downMost = compVec[i]->get_ll_y();
	}

	/* move whole dieArea and macros to the new origin*/
	if (leftMost < 0) {

		originDisplacement.first = -leftMost;
		for (size_t i = 0; i < dieArea.size(); ++i)
			dieArea[i].first += originDisplacement.first;
		for (size_t i = 0; i < compVec.size(); ++i) {
			compVec[i]->_llx += originDisplacement.first;
			compVec[i]->_originX += originDisplacement.first;
		}
		minOfWidth += originDisplacement.first;
		maxOfWidth += originDisplacement.first;
	}
	else {
		// if original origin has been non-negative, then do nothing
		originDisplacement.first = 0;
	}

	if (downMost < 0) {
		originDisplacement.second = -downMost;
		for (size_t i = 0; i < dieArea.size(); ++i)
			dieArea[i].second += originDisplacement.second;
		for (size_t i = 0; i < compVec.size(); ++i) {
			compVec[i]->_lly += originDisplacement.second;
			compVec[i]->_originY += originDisplacement.second;
		}
		minOfHeight += originDisplacement.second;
		maxOfHeight += originDisplacement.second;
	} 
	else {
		// if original origin has been non-negative, then do nothing
		originDisplacement.second = 0;
	}
}

/*
*
* recover coordinate system after all works done and before write def
*
*/
void Parser::recoverCoordinateSystem(vector<Component*>& compVec)
{
	if (originDisplacement.first == 0) 
		return;
	else {
		for (size_t i = 0; i < dieArea.size(); ++i)
			dieArea[i].first -= originDisplacement.first;
		for (size_t i = 0; i < compVec.size(); ++i) {
			compVec[i]->_llx -= originDisplacement.first;
			compVec[i]->_originX -= originDisplacement.first;
		}
		minOfWidth -= originDisplacement.first;
		maxOfWidth -= originDisplacement.first;
	}

	if (originDisplacement.second == 0)
		return;
	else {
		for (size_t i = 0; i < dieArea.size(); ++i)
			dieArea[i].second -= originDisplacement.second;
		for (size_t i = 0; i < compVec.size(); ++i) {
			compVec[i]->_lly -= originDisplacement.second;
			compVec[i]->_originY -= originDisplacement.second;
		}
		minOfHeight -= originDisplacement.second;
		maxOfHeight -= originDisplacement.second;
	}
}