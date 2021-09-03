#include "cluster.h"
using namespace bc;

void BestChoiceCluster::initGroupForEachComp()
{
    for (vector<Component*>::iterator it = _compVec.begin(); it != _compVec.end(); ++it) {
        Group *group = new Group();
        group->_cgx = (*it)->get_mid_x();
        group->_cgy = (*it)->get_mid_y();
        group->_members.push_back(*it);
        group->_macro = (*it)->macro;
        group->_token = (*it)->nameToken;
        group->_totalArea = (*it)->getArea();
        _groups.push_back(group);
    }
}

/*
*bigger is better 
*/
double BestChoiceCluster::evalCost(Group *g1, Group *g2)
{
    /* score function - parameters */
    double paraD = 1.0;
    double paraH = 0.0;
    double paraA = 0.0;

    /* score function - distance */
    int dx = abs(g1->_cgx - g2->_cgx);
    int dy = abs(g1->_cgy - g2->_cgy);
    double distance = (double)dx + (double)dy;
    double inverseDistance = distance == 0? DBL_MAX : 1.0 / distance;

    /* score function - common hierarchies */
    std::set<std::string> commonToken;
	std::set_intersection(  g1->_token.begin(), g1->_token.end(), 
                            g2->_token.begin(), g2->_token.end(),
                            std::inserter(commonToken, commonToken.begin()));
    double hierarchies = commonToken.size();

    /* score function - area difference */
    double avgArea1 = g1->_totalArea / g1->_members.size();
    double avgArea2 = g2->_totalArea / g2->_members.size();
    double diffArea = fabs(avgArea1 - avgArea2) / (avgArea1 + avgArea2);
    double inverseDiffArea = diffArea == 0? DBL_MAX : 1.0 / diffArea;

    /* final weighted score function */
    return paraD * distance + paraH * hierarchies + paraA * inverseDiffArea;
}

void BestChoiceCluster::initPriorityQueue()
{
    /* for each group */
    for (size_t i = 0; i < _groups.size(); ++i ) {
        /* find the closest group with it */
        int minDistance = INT_MAX;  // Manhattan distance
        Group *closest = NULL;         // the closest group with it
        for (size_t j = 0; j < _groups.size(); ++j) {

            // don't compare with itself
            if (_groups[i] == _groups[j]) continue;
            // only compare with the group belongs to the same macro 
            if (_groups[i]->_macro != _groups[j]->_macro) continue;

            // Manhattan distance
            int distance = abs(_groups[i]->_cgx - _groups[j]->_cgx) + abs(_groups[i]->_cgy - _groups[j]->_cgy);

            if (distance < minDistance) {
                minDistance = distance;
                closest = _groups[j];
            }
        }

        /* if this macro type has more than one components */
        if (closest != NULL) {
            //cout << i << ": " << _groups[i]->_macro->name << " " << _groups[closest]->_macro->name << endl;
            double cost = evalCost(_groups[i], closest);
            Tuple tuple = make_pair(make_pair(_groups[i], closest), cost);
            _PQ.push(tuple);
        }
    }

    /*while (!_PQ.empty()) {
        Tuple tuple = _PQ.top();
        _PQ.pop();
        cout << "cost: " << tuple.second << endl; 
    }*/
}

Group* BestChoiceCluster::getClostestGroup(Group* target)
{
    int minDistance = INT_MAX;  // Manhattan distance
    Group* closestGroup = NULL;           // the closest group's index with it
    for (size_t i = 0; i < _groups.size(); ++i) {

        // don't compare with itself || don't compare with the invalid group
        if (_groups[i] == target || !_groups[i]->_valid)
            continue;
        // only compare with the group belongs to the same macro
        if (target->_macro != _groups[i]->_macro)
            continue;

        // Manhattan distance
        int distance = abs(target->_cgx - _groups[i]->_cgx) + abs(target->_cgy - _groups[i]->_cgy);

        if (distance < minDistance) {
            minDistance = distance;
            closestGroup = _groups[i];
        }
    }

    return closestGroup;
}

Group* BestChoiceCluster::clusterGroup(Group *g1, Group *g2)
{
    Group *cluster = new Group();
    /* concatenate the members from g1 and g2 into cluster's menber */
    cluster->_members.reserve(g1->_members.size() + g2->_members.size()); // preallocate memory
    cluster->_members.insert(cluster->_members.end(), g1->_members.begin(), g1->_members.end());
    cluster->_members.insert(cluster->_members.end(), g2->_members.begin(), g2->_members.end());

    /* union two commonTokens */
    std::set_union( g1->_token.begin(), g1->_token.end(),
                    g2->_token.begin(), g2->_token.end(),
                    std::inserter(cluster->_token, cluster->_token.begin()));

    /* calculate the total after clustered */
    cluster->_totalArea = g1->_totalArea + g2->_totalArea;

    /* calculate the center of gravity x/y */
    /* gravity x/y = total center postion / size of members */
    int cgx = 0, cgy = 0;
    for (size_t i = 0; i < cluster->_members.size(); ++i) {
        cgx += cluster->_members[i]->get_mid_x();
        cgy += cluster->_members[i]->get_mid_y();
    }
    cgx /= cluster->_members.size();
    cgy /= cluster->_members.size();

    cluster->_cgx = cgx;
    cluster->_cgy = cgy;

    cluster->_macro = g1->_macro;

    g1->_valid = false;
    g2->_valid = false;
    _groups.erase(std::find(_groups.begin(), _groups.end(), g1));
    _groups.erase(std::find(_groups.begin(), _groups.end(), g2));

    _groups.push_back(cluster);

    return cluster;
}

bool BestChoiceCluster::satisfyHardConstraints(Group *g1, Group *g2)
{
    /* parameters */
    double paraA = 0.09;

    /* the area constraint */
    if (g1->_totalArea + g2->_totalArea > paraA * _designArea)
        //return false;
    
    return true;
}

void BestChoiceCluster::bestChoiceClustering()
{
    /* Phase I. Priority-queue PQ Initialization */
    initGroupForEachComp();
    initPriorityQueue();

    /* Phase II. Clustering */
    _designArea = (double)_designWidth * (double)_designHeight;
    // target clustering ratio
    int alpha = 10;
    int numOfGroups = 0;
    int targetNumOfGroups = _compVec.size() / alpha;

    //for (int round = 0; round < 1000; ++round)
    //while (numOfGroups > targetNumOfGroups) {
    //while (_PQ.top().second > 6.0) {
    cout << "_designWidth: " << _designWidth << endl;
    cout << "top: " << _PQ.top().second << " " << (double)_designWidth * 0.3 << endl;
    while (_PQ.top().second < (double)_designWidth * 0.06) {
        if (_PQ.empty())
            break;

        Group* group1 = _PQ.top().first.first;
        Group* group2 = _PQ.top().first.second;

        cout << "top: " << group1->_macro->name << " " << group2->_macro->name << endl;
        cout << "score: " << _PQ.top().second << endl;

        _PQ.pop();

        /* if both groups are invalid */
        if (!group1->_valid && !group2->_valid) {}
        /* if both groups are valid */
        else if (group1->_valid && group2->_valid) {

            /*if (!satisfyHardConstraints(_groups[index1], _groups[index2]))
                continue;*/
            
            Group* newCluster = clusterGroup(group1, group2);

            Group* clostest = getClostestGroup(newCluster);
            
            if (clostest == NULL)
                continue;

            double cost = evalCost(newCluster, clostest);
            Tuple tuple = make_pair(make_pair(newCluster, clostest), cost);
            _PQ.push(tuple);
            numOfGroups ++;
        }
        /* either one is invalid */
        else {
            Group* valid = group1->_valid ? group1 : group2;
            Group* clostest = getClostestGroup(valid);

            if (clostest == NULL)
                continue;

            double cost = evalCost(valid, clostest);
            Tuple tuple = make_pair(make_pair(valid, clostest), cost);
            _PQ.push(tuple);
        }
    }
}

/*void BestChoiceCluster::printGroups()
{
    for (size_t i = 0; i < _groups.size(); ++i) {
        cout << "group " << i << ": ";

        if(_groups[i] == NULL) cout << "NULL";
        else {
            for (size_t j = 0; j < _groups[i]->_members.size(); ++j) {
                cout <<  _groups[i]->_members[j]->getName() << " ";
            }
        }
        cout << endl;
    }
}

void BestChoiceCluster::printPriorityQueue()
{
    while (!_PQ.empty()) {
        size_t index1 = _PQ.top().first.first;
        size_t index2 = _PQ.top().first.second;
        cout << _groups[index1]->_members[0]->getName() << "\t" << _groups[index2]->_members[0]->getName() << "\t" << _PQ.top().second << endl;
        _PQ.pop();
    }
}*/
