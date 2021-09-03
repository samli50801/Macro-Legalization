#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <math.h>
#include <time.h>

using namespace std;

class Component{
public:
	string name;
	string type;
	string status;
	int x;
	int y;
};

class type{
public:
	double width;
	double height;
	int num;
};


void readlef(char *argc[], map<string, type> &macroTypes){
	fstream flef, fout;
	string token;
	flef.open(argc[2] , ios::in);
	fout.open("./newCase.lef", ios::out);
	if(!flef)
		cout << "Can't open lef file" << endl;
	while(getline(flef,token))
		fout << token << endl;
	flef.close();

	flef.open(argc[2] , ios::in);
	if(!flef)
		cout << "Can't open lef file" << endl;
	while(flef >> token){
		string  name;
		type temp_t;
		flef >> name;
		if(name == "LIBRARY")
			break;
		flef >> token >> temp_t.width >> token >> temp_t.height >> token;
		flef >> token >> token;
		temp_t.num = 0;
		macroTypes[name] = temp_t;
	}
	flef.close();
	return;
}

int readdef(char *argc[], vector<Component> &comps, map<string, type> &macroTypes){
	fstream fdef, fout;
	fdef.open(argc[1] , ios::in);
	string token, designName, udm;
	if(!fdef)
		cout << "Can't open def file" << endl;
	fdef >> token >> token >> token;
	fdef >> token >> designName >> token;
	fdef >> token >> token >> token >> udm >> token;
	while(token != "COMPONENTS")
		fdef >> token;

	fdef >> token >> token;
	
	while(fdef >> token){
		if(token == "DESIGN") break;

		if(token == "-"){
			Component temp_comp;
			fdef >> token;
			temp_comp.name = "macro_" + to_string(comps.size()+1);
			fdef >> temp_comp.type;
			macroTypes[temp_comp.type].num++;
			fdef >> token >> temp_comp.status >> token >> temp_comp.x >> temp_comp.y >> token >> token >> token;
			comps.push_back(temp_comp);
		}
	}
	return stoi(udm);
}

void readtxt(char *argc[]){
	fstream ftxt, fout;
	ftxt.open(argc[3], ios::in);
	fout.open("./newCase.txt", ios::out);
	if(!ftxt)
		cout << "Can't open txt file" << endl;
	string token;
	while(getline(ftxt,token))
		fout << token << endl;
}

int extend(char *argc[], vector<Component> &comps, map<string, type> &macroTypes, int udm){
	int need = stoi(argc[4]) / macroTypes.size();
	int start = comps.size();
	int totalArea = 0;
	for(auto i = macroTypes.begin(); i != macroTypes.end(); i++){
		while(i->second.num < need){
			Component temp_comp;
			temp_comp.name = "macro_" + to_string(comps.size()+1);
			temp_comp.type = i->first;
			temp_comp.status = "PLACED";
			comps.push_back(temp_comp);
			i->second.num++;
		}
		totalArea += i->second.width * i->second.height * i->second.num;
	}
	cout << "total area: " << totalArea << endl;
	int side = ceil(sqrt(totalArea / stod(argc[5]))) * udm;
	srand(time(NULL));
	for(int i = start; i < comps.size(); i++){
		comps[i].x = rand() % side;
		comps[i].y = rand() % side;
	} 
	return side;
}

void output(vector<Component>comps, int side, int udm){
	fstream fout;
	fout.open("./newCase.def", ios::out);
	fout << "VERSION 5.7 ;" << endl;
	fout << "DESIGN newCase ;" << endl;
	fout << "UNITS DISTANCE MICRONS " << udm << " ;" << endl << endl;
	fout << "DIEAREA ( 0 0 ) ( " << side << " " << side << " ) ;" << endl << endl;
	fout << "COMPONENTS " << comps.size() << " ;"<< endl;
	for(int i = 0; i < comps.size(); i++){
		fout << "- " << comps[i].name << " " << comps[i].type << " " << endl << "       + " << comps[i].status << " ( " << comps[i].x  << " " << comps[i].y << " ) N ;" << endl;
	}
	fout << "END COMPONENTS" << endl << endl << endl << endl;
	fout << "END DESIGN" << endl << endl;
	fout.close();
}

int main(int argv, char *argc[]){
	
	vector<Component> comps;
	map<string, type> macroTypes;
	int side, udm;
	readlef(argc,macroTypes);
	udm = readdef(argc,comps, macroTypes);
	readtxt(argc);
	side = extend(argc, comps, macroTypes, udm);
	output(comps, side, udm);

	/*
	int count = 0;
	cout << "---------------" << endl;
	for(auto i = macroTypes.begin(); i != macroTypes.end(); i++){
		cout << i->first << " " << i->second.width << " " << i->second.height << " " << i->second.num << endl;
		count += i->second.num;
	}
	cout << "--------------" << endl;
	for(int i = 0; i < comps.size(); i++){
		cout << comps[i].name << " " << comps[i].type << " " << comps[i].status << " " << comps[i].x << " " << comps[i].y << endl;
	}
	cout << "--------------" << endl;
	cout << "type count: " << macroTypes.size() << " total count: " << count << endl;
	cout << "comps size: " <<comps.size() << endl;
*/
	
	return 0;
}
