#include "SweepLine.h"

bool 
IntervalTree::ifOverlap(Node* i, Node* j, const int minSpace)
{
	if ((*i).comp->isBoundary && (*j).comp->isBoundary) 
		return false;

	double x_overlap = (i->getLength() + j->getLength()) / 2.0 - fabs(i->getPos() - j->getPos());
	double y_overlap = (i->getComp()->height + j->getComp()->height) / 2.0 - fabs(i->getComp()->_cy - j->getComp()->_cy);

	double shrink_x_overlap = x_overlap - minSpace;
	double shrink_y_overlap = y_overlap - minSpace;

	if ((*i).comp->isBoundary || (*j).comp->isBoundary) {
		x_overlap -= minSpace / 2.0;
		y_overlap -= minSpace / 2.0;
	}
	
	if (shrink_x_overlap <= EPSILON &&
		shrink_y_overlap <= EPSILON) 
		return false;

	if (x_overlap > EPSILON && 
		y_overlap > EPSILON) {
		//DEBUG("i->getWidth() + j->getWidth(): " << (i->getWidth() + j->getWidth()) / 2 << " fabs(i->getPos() - j->getPos()): " << fabs(i->getPos() - j->getPos())<< " x_overlap: " << x_overlap << endl);
		return true;
	}

	return false;
}

bool 
IntervalTree::ifOverlap_for_overlap_group(Node* i, Node* j)
{
	if ((*i).comp->isBoundary && (*j).comp->isBoundary) 
		return false;

	/* lower-left x   lower-left y   upper-right x   uppre-right y */
	int rec1[4] = { i->comp->get_ll_x(), i->comp->get_ll_y(), i->comp->get_ur_x(), i->comp->get_ur_y() };
	int rec2[4] = { j->comp->get_ll_x(), j->comp->get_ll_y(), j->comp->get_ur_x(), j->comp->get_ur_y() };

	int shrinkRec1[4] = { rec1[0] + _minSpace / 2, rec1[1] + _minSpace / 2, rec1[2] - _minSpace / 2, rec1[3] - _minSpace / 2 };
	int shrinkRec2[4] = { rec2[0] + _minSpace / 2, rec2[1] + _minSpace / 2, rec2[2] - _minSpace / 2, rec2[3] - _minSpace / 2 };

	if (!(*i).comp->isBoundary && !(*j).comp->isBoundary) { // Both are macros
	
		// avoid to misjudge such a situation as overlap
		//
		//			*****
		//			*   *
		//			*   *
		//			*****
		//				 *****
		//				 *   *
		//				 *   *
		//				 *****
		//		two non-expanded macro		
		if ((shrinkRec1[0] >= shrinkRec2[2] || shrinkRec2[0] >= shrinkRec1[2]) && (shrinkRec1[1] >= shrinkRec2[3] || shrinkRec2[1] >= shrinkRec1[3]))
			return false;

	}else { // One of the nodes is boundary

		if (!i->comp->isBoundary)
			std::copy(shrinkRec1, shrinkRec1 + 4, rec1);
		else
			std::copy(shrinkRec2, shrinkRec2 + 4, rec2);
	}

	if (rec1[0] < rec2[2] && rec2[0] < rec1[2] && rec1[1] < rec2[3] && rec2[1] < rec1[3])
		return true;
	else
		return false;
}

bool
IntervalTree::ifFreeSpaceOverlap(Node* i, Node* j, bool direction)
{
	/* if two interval overlap */
	//cout << "i->getHigh(): " << i->getHigh() << " j->getLow(): " << j->getLow() << " j->getHigh(): " << j->getHigh() << " i->getLow(): " 
	//<<  i->getLow() << endl;
	if (i->getHigh() < j->getLow() || j->getHigh() < i->getLow())
		return false; 
	return true;
}

int 
IntervalTree::height(Node *N)
{
	if (N == NULL)
		return 0;
	return N->height;
}

int 
IntervalTree::maxHeight(Node* node)
{
	return (height(node->left) > height(node->right)) ? height(node->left) : height(node->right);
}

int 
IntervalTree::getBalance(Node* node)
{
	if (node->left == NULL && node->right == NULL)
		return 0;
	if (node->left == NULL)
		return 0 - node->right->height;
	else if (node->right == NULL)
		return node->left->height;
	else
		return node->left->height - node->right->height;
}

int 
IntervalTree::maxValue(Node* node)
{
	if (node->left == NULL && node->right == NULL)
		return 0;
	else if (node->left == NULL)
		return node->right->max;
	else if (node->right == NULL)
		return node->left->max;
	else
		return (node->left->max > node->right->max) ? node->left->max : node->right->max;
}

void 
IntervalTree::updateMaxVaue(Node *node)
{
	if (node == NULL) {
		DEBUG("updateMaxValue: node is NULL\n");
		return;
	}

	if (node->left == NULL && node->right == NULL) {
		node->setMax(node->high);
		return;
	}
	else if (node->left == NULL) 
		node->setMax(node->right->max);
	else if (node->right == NULL)
		node->max = node->left->max;
	else if (node->left->max > node->right->max)
		node->max = node->left->max;
	else
		node->max = node->right->max;

	// Update Max Value
	if (node->high > node->max) {
		node->max = node->high;
	}

}

Node* 
IntervalTree::minValueNode(Node *node)
{
	Node* current = node;

	while (current->getLeft() != NULL)
		current = current->getLeft();

	return current;
}

Node* 
IntervalTree::rightRotate(Node* y)
{
	Node *x = y->getLeft();
	Node *T2 = x->getRight();

	//perform rotation
	y->setLeft(T2);
	x->setRight(y);

	//update heights
	y->height = 1 + ((height(y->getLeft()) > height(y->getRight())) ? height(y->getLeft()) : height(y->getRight()));
	x->height = 1 + ((height(x->getLeft()) > height(x->getRight())) ? height(x->getLeft()) : height(x->getRight()));

	// Update Max Value
	updateMaxVaue(y);
	updateMaxVaue(x);

	// Return new root
	return x;
}

Node*
IntervalTree::leftRotate(Node *_cx)
{
	Node *_cy = _cx->right;
	Node *T2 = _cy->left;

	// Perform rotation
	_cy->left = _cx;
	_cx->right = T2;

	// Update heights
	_cx->height = 1 + ((height(_cx->left) > height(_cx->right)) ? height(_cx->left) : height(_cx->right));
	_cy->height = 1 + ((height(_cy->left) > height(_cy->right)) ? height(_cy->left) : height(_cy->right));

	// Update Max Value
	updateMaxVaue(_cx);
	updateMaxVaue(_cy);

	// Return new root
	return _cy;
}

void 
IntervalTree::search_overlap(vector<Component*>& o_comp, Node *root, Node *n, const int minSpace)
{
	if (root == NULL)
		return;

	if (ifOverlap_for_overlap_group(root, n)) {
		o_comp.push_back(root->getComp());
	}

	if (root->left != NULL && root->left->max > n->low)
		search_overlap(o_comp, root->getLeft(), n, minSpace);

	if (root->getRight() != NULL)
		search_overlap(o_comp, root->getRight(), n, minSpace);
}

void
IntervalTree::search_overlap_for_overlap_group(vector<Component*>& o_comp, Node *root, Node *n)
{
	if (root == NULL)
		return;

	if (ifOverlap_for_overlap_group(root, n)) {
		o_comp.push_back(root->getComp());
	}

	if (root->left != NULL && root->left->max > n->low)
		search_overlap_for_overlap_group(o_comp, root->getLeft(), n);

	if (root->getRight() != NULL)
		search_overlap_for_overlap_group(o_comp, root->getRight(), n);
}

void 
IntervalTree::print_infix(Node* root)
{
	if (root == NULL) return;

	if (root->left != NULL)
		print_infix(root->left);
	if (root->right != NULL)
		print_infix(root->right);
	cout << root->low << "~" << root->high << ", max: " << root->max << ", height: " << root->height 
	<< ", balance: " << getBalance(root) << endl;
}

Node* 
IntervalTree::updateTreeBalance(Node* current)
{
	int balance = getBalance(current);

	if (balance > 1) {
		// Left Left Case
		if (current->getLeft() != NULL && current->getLeft()->getLeft() != NULL) {
			return rightRotate(current);
		}
		// Left Right Case
		else {
			current->setLeft(leftRotate(current->getLeft()));
			return rightRotate(current);
		}
	}
	else if (balance < -1) {
		// Right Right Case
		if (current->getRight() != NULL && current->getRight()->getRight() != NULL) {
			return leftRotate(current);
		}
		// Right Left Case
		else {
			current->setRight(rightRotate(current->getRight()));
			return leftRotate(current);
		}
	}

	return current;
}

Node* 
IntervalTree::insert(Node* current, Node *add)
{
	if (current == NULL)
		return add;

	if (add->getLow() >= current->getLow())
		current->setRight(insert(current->getRight(), add));
	else
		current->setLeft(insert(current->getLeft(), add));

	if (add->getMax() > current->getMax())
		current->setMax(add->getMax());

	// update tree height
	current->setHeight(1 + maxHeight(current));
	// update tree balance
	current = updateTreeBalance(current);

	/*
	// Left Left Case
	if (balance > 1 && add->getLow() < current->getLeft()->getLow())
		return rightRotate(current);

	// Right Right Case
	if (balance < -1 && add->low > current->getRight()->getLow())
		return leftRotate(current);

	// Left Right Case
	if (balance > 1 && add->getLow() > current->getLeft()->getLow()) {
		current->setLeft(leftRotate(current->getLeft()));
		return rightRotate(current);
	}

	// Right Left Case
	if (balance < -1 && add->getLow() < current->getRight()->getLow())
	{
		current->setRight(rightRotate(current->getRight()));
		return leftRotate(current);
	}*/

	return current;
}

Node* 
IntervalTree::deleteNode(Node *root, Node *node)
{
	if (root == NULL) 
		return root;

	if (node->getLow() < root->getLow())
		root->setLeft(deleteNode(root->getLeft(), node));
		
	else if (node->getLow() > root->getLow()) 
		root->setRight(deleteNode(root->getRight(), node));
	else {

		// node with only one child or no child
		if (root->getRight() == NULL ||
			root->getLeft() == NULL)
		{
			Node *temp = root->left ? root->left : root->right;

			// No child case
			if (temp == NULL)
			{
				temp = root;
				root = NULL;
			}
			else // One child case
				*root = *temp;

			delete temp;
		}
		else
		{
			// node with two children: Get the inorder 
			// successor (smallest in the right subtree) 
			Node* temp = minValueNode(root->getRight());

			// Copy the inorder successor's 
			// data to this node
			root->setLow(temp->getLow());
			root->setHigh(temp->getHigh());
			root->setPos(temp->getPos());
			root->setComp(temp->getComp());
			root->setBegin(temp->getBegin());

			// Delete the inorder successor 
			root->setRight(deleteNode(root->getRight(), temp));
		}
	}

	if (root == NULL)
		return root;

	// STEP 2: update tree height
	root->setHeight(1 + max(height(root->left), height(root->right)));
	// update tree balance
	return updateTreeBalance(root);
	/*
	int balance = getBalance(root);

	// Left Left Case 
	if (balance > 1 &&
		getBalance(root->getLeft()) >= 0)
		return rightRotate(root);

	// Left Right Case 
	if (balance > 1 &&
		getBalance(root->getLeft()) < 0)
	{
		root->setLeft(leftRotate(root->getLeft()));
		return rightRotate(root);
	}

	// Right Right Case 
	if (balance < -1 &&
		getBalance(root->getRight()) <= 0)
		return leftRotate(root);

	// Right Left Case 
	if (balance < -1 &&
		getBalance(root->getRight()) > 0)
	{
		root->setRight(rightRotate(root->getRight()));
		return leftRotate(root);
	}

	return root;*/
}

Node*
IntervalTree::get_interval(Node* root, Node *n, bool direction)
{
	if (root == NULL)
		return NULL;
	//cout << "root: " << root->low << " " << root->high << endl;
	//cout << "node: " << n->low << " " << n->high << endl;

	if (ifFreeSpaceOverlap(root, n, direction)) {
		DEBUG("overlap\n");
		//cout << "overlap: " << root->getLow() << " " << root->getHigh() << endl;
		return root;
	}
	else if (root->left != NULL && root->left->max > n->low) {
		DEBUG("root->left: " << root->left->low << " " << root->left->high << endl);
		get_interval(root->getLeft(), n, direction);
	}
	else if (root->getRight() != NULL) {
		DEBUG("root->right: " << root->getRight()->getLow() << " " << root->getRight()->getHigh() << endl);
		get_interval(root->getRight(), n, direction);
	}
}

Node*
IntervalTree::find_left_adjacent_interval(Node* root, int point)
{
	if (root == NULL) return NULL;

	if (root->getHigh() == point)
		return root;
	else if (root->getLeft() != NULL && root->getLow() > point)
		find_left_adjacent_interval(root->left, point);
	else if (root->getRight() != NULL)
		find_left_adjacent_interval(root->getRight(), point);
}
Node*
IntervalTree::find_right_adjacent_interval(Node* root, int point)
{
	if (root == NULL) return NULL;

	if (root->getLow() == point)
		return root;
	else if (root->getRight() != NULL && root->getLow() < point)
		find_right_adjacent_interval(root->getRight(), point);
	else if (root->getLeft() != NULL)
		find_right_adjacent_interval(root->getLeft(), point);
}

bool
IntervalTree::ifCluster(Node* i, Node* j)
{
	double x_overlap = (i->getLength() + j->getLength()) * 0.5 - fabs(i->getPos() - j->getPos());

	/*DEBUG("x_overlap: " << x_overlap << endl);
	DEBUG("i->getPathPos(): " << i->getPathPos() << " / " << "j->getPathPos(): " << j->getPathPos() << endl);*/

	if (x_overlap >= 0 && fEq(x_overlap, i->getLength()))
		return true;
	else if (x_overlap >= 0 && fEq(i->getPathPos(), j->getPathPos()))
		return true;

	return false;
}