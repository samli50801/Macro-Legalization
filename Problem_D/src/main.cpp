#include <ctime>
#include "Force_Directed.h"
#include "FreeSpace.h"
#include "preprocessor.h"
#include "tile.h"
#include "legalizer.h"
#include "cluster.h"
#include "CG.h"
#include <time.h> // includes clock_t and CLOCKS_PER_SEC

//ps_context *context; // Draw Design
Parser parser;

map<string, Macro*> macro;					// All Macro

vector<Component*> comp;					// All component

unordered_set<Component*> free_space;		// All the space which can place standard cell
unordered_set<Component*> not_free_space;	// All the space which can't place standard cell
vector<Component*> boundaryComp;

vector<Bound*> bound;						// Segments of the design boundary

void plot(Rectangle);
											
vector<vector<Component*>> overlapGroup;	// overlapped group for corner-stitching
vector<bc::Group*> groups;					// clustering groups

bool isInOverlapGroup(Component* target) {
	for(vector<vector<Component*>>::iterator it = overlapGroup.begin(); it != overlapGroup.end(); ++it){
		for (vector<Component*>::iterator i = (*it).begin(); i != (*it).end(); ++i){
			if (target == *i)
				return true;
		}
	}
	return false;
}
vector<Component*> deletePoint;
vector<Component*> movedMacro;

int main(int argc, char* argv[])
{
	/* record time*/
	clock_t start, end;
	double duration_sec;
	fstream Log("log.txt", ios::out);

    fstream output;
    output.open(argv[4], ios::out);

	parser.readLef("testbench/" + string(argv[2]), macro);
	parser.readDef("testbench/" + string(argv[1]), output, macro, comp);
	parser.readTxt("testbench/" + string(argv[3]));
	parser.moveCoordinateSystem(comp);

	parser.load_bounding();
	bound = parser.getBound();

	/* preprcessor: bounding box and boundary tiles construction */
    Preprocessor preprocessor(bound, parser);
    preprocessor.run();
	//preprocessor.buildBoundaryComponent();
	boundaryComp = preprocessor.getBoundaryComp();
	
	// For corner-stitching
	Force_Directed fd(parser._mcsbmc * parser.dbuPerMicron);
	fd.move_macro_inside_boundary();	// Move macros ounside the design into the nearest boundary
	start = clock();	// time
	fd.do_forceDirected();
	end = clock();	// time
	duration_sec = double(end-start) / CLOCKS_PER_SEC;	// time
	Log << "Force-directed: " << duration_sec << "sec" << endl;

	bc::BestChoiceCluster cluster(comp, parser);
	cluster.setDesignWidth(parser.maxOfWidth - parser.minOfWidth);
	cluster.setDesignHeight(parser.maxOfHeight - parser.minOfHeight);
	cluster.setLongestTokenLength(parser.longestTokenLength);
	//cluster.bestChoiceClustering();
	//cluster.printGroups();
	//groups = cluster.getGroups();

	cg::CG cg(comp, boundaryComp, parser);
	cg.legalize();
	deletePoint = cg.deletedPoint;		// Debug
		

    //plot(preprocessor.getBoundingRect());
    //parser.score_evaluation(comp, not_free_space);

    Legalizer legalizer(comp, 
            preprocessor.getPlane(), 
            preprocessor.getBoundingRect(),
            parser._mcsbmc * parser.dbuPerMicron);

	start = clock();	// time
	legalizer.legalize();
	end = clock();	// time
	duration_sec = double(end-start) / CLOCKS_PER_SEC;	// time
	Log << "corner-stitch: " << duration_sec << "sec" << endl;
    //legalizer.plot();

	FreeSpace fs(comp, bound, parser);
	fs.findFreeSpace();
	free_space = fs.getFreeSpace();
	not_free_space = fs.getNotFreeSpace();

	/*check if the final placement is legal (no overlappings) */
	fd.generate_overlap_group();		// Generate overlapped group
	overlapGroup = fd.getGroup();
	if (overlapGroup.empty())
		cout << "Successfully legalize\n";
	else 
		cout << "Fail to legalize\n";

	/* gnuplot */
	//plot(preprocessor.getBoundingRect());

	parser.recoverCoordinateSystem(comp);
	parser.writeDef(output, comp);

    parser.score_evaluation(comp, not_free_space);

	/* gnuplot */
	plot(preprocessor.getBoundingRect());

	return 0;
}

void plot(Rectangle boundingRect){
	/////////////info. to show for gnu/////////////
    int boundWidth = parser.maxOfWidth - parser.minOfWidth;// user-define value (boundary info)
    int boundHeight = parser.maxOfHeight - parser.minOfHeight;// same above
 	/////////////////////////////////////////////
 	//gnuplot preset
	fstream outgraph("output.gp", ios::out);
	outgraph << "reset\n";
	outgraph << "set tics\n";
	outgraph << "unset key\n";
	outgraph << "set title \"The result of Floorplan\"\n";
	int index = 1;
 	// wirte block info into output.gp

	string ColorValues[40] = {
	"#000000", "#ff0000", "#00c000", "#0080ff", "#c000ff", "#ff8040", "#c04000", "#c8c000", "#4169e1", "#ffc020",
	"#008040", "#c080ff", "#306080", "#8b0000", "#408000", "#ff80ff", "#a52a2a", "#ffff00", "#40e0d0", "#f03232",
	"#90ee99", "#f055f0", "#ffd700", "#00ff00", "#006400", "#0000ff", "#000080", "#00ffff", "#ff00ff", "#00ced1",
	"#ff1493", "#ff7f50", "#ff4500", "#9400d3", "#905040", "#556b2f", "#801400", "#804080", "#8060ff", "#808000"
	};
	/*size_t colorIndex = 0;
	for (size_t i = 0; i < groups.size(); ++i) {
		if (groups[i] != NULL) {
			for (size_t j = 0; j < groups[i]->_members.size(); ++j) {
				Component c = *groups[i]->_members[j];
                string NodeName = c.getName();
	    		int x0 = c.get_ll_x();
	    		int y0 =  c.get_ll_y();
				int x1 = c.get_ur_x();
				int y1 = c.get_ur_y();
				int midX = (x0+x1)/2;
				int midY = (y0+y1)/2;
				if (groups[i]->_members.size() == 1) {
					outgraph << "set object " << index << " rect from "
					<< x0 << "," << y0 << " to " << x1 << "," << y1 << " fs empty border lc rgb '#000000'\n";
					index++;
				}else {
					outgraph << "set object " << index << " rect from " 
		  			<< x0 << "," << y0 << " to " << x1 << "," << y1 << " fs empty border lc rgb '" << ColorValues[colorIndex % 40] << "'\n";
					index++;
				}
            }
			colorIndex++;
		}
	}*/
 	for (vector<Component*>::iterator i = comp.begin(); i != comp.end(); i++) {

		string NodeName = (*i)->getName();
	    int x0 = (*i)->get_ll_x();
	    int y0 =  (*i)->get_ll_y();
		int x1 = (*i)->get_ur_x();
		int y1 = (*i)->get_ur_y();
		int midX = (x0+x1)/2;
		int midY = (y0+y1)/2;

		/*if (isInOverlapGroup(*i)){
			outgraph << "set object " << index << " rect from " 
		  		<< x0 << "," << y0 << " to " << x1 << "," << y1 << " fs empty border lc rgb '#ff0000'\n";
				//<< "set label " << "\"" << NodeName << "\"" << " at " << midX << "," << midY << " center " << "font \",8\"\n";
		}else {*/
			if ((*i)->getType() == FIXED) {
				outgraph << "set object " << index << " rect from " 
		  		<< x0 << "," << y0 << " to " << x1 << "," << y1 << " fs empty border lc rgb 'green'\n";
			} else {
				outgraph << "set object " << index << " rect from " 
		  		<< x0 << "," << y0 << " to " << x1 << "," << y1 << " fs empty border lc rgb '#000000'\n";
				//<< "set label " << "\"" << NodeName << "\"" << " at " << midX << "," << midY << " center " << "font \",8\"\n";
			}
		/*}*/
		
		index++;
	}
	/*size_t colorIndex = 5;
	int c = 1;
	for (vector<Component*>::iterator i = movedMacro.begin(); i != movedMacro.end(); i++) {
		string NodeName = (*i)->getName();
	    int x0 = (*i)->get_ll_x();
	    int y0 =  (*i)->get_ll_y();
		int x1 = (*i)->get_ur_x();
		int y1 = (*i)->get_ur_y();
		int midX = (x0+x1)/2;
		int midY = (y0+y1)/2;
		outgraph << "set object " << index << " rect from " 
		  		<< x0 << "," << y0 << " to " << x1 << "," << y1 << " fs empty border lc rgb '" << ColorValues[colorIndex % 40] <<  "'\n";
			//<< "set label " << "\"" << NodeName << "\"" << " at " << midX << "," << midY << " center " << "font \",8\"\n";
	
		index++;
		if (c%2 == 0) colorIndex++;
		c++;
	}*/
	for (vector<Component*>::iterator i = deletePoint.begin(); i != deletePoint.end(); i++) {
		string NodeName = (*i)->getName();
	    int x0 = (*i)->get_ll_x();
	    int y0 =  (*i)->get_ll_y();
		int x1 = (*i)->get_ur_x();
		int y1 = (*i)->get_ur_y();
		int midX = (x0+x1)/2;
		int midY = (y0+y1)/2;
		
		outgraph << "set object " << index << " rect from " 
		  	<< x0 << "," << y0 << " to " << x1 << "," << y1 << " fs empty border lc rgb '#0080ff'\n";
			//<< "set label " << "\"" << NodeName << "\"" << " at " << midX << "," << midY << " center " << "font \",8\"\n";
	
		index++;  
	}
	
	/* boundary component*/
	/*for (auto it = boundaryComp.begin(); it != boundaryComp.end(); ++it) {
		int x0 = (*it)->get_ll_x();
	    int y0 =  (*it)->get_ll_y();
		int x1 = (*it)->get_ur_x();
		int y1 = (*it)->get_ur_y();
		cout << x0 << " " << y0 << " " << x1 << " " << y1 << endl;
		int midX = (x0+x1)/2;
		int midY = (y0+y1)/2;
		outgraph << "set object " << index << " rect from " 
		  	<< x0 << "," << y0 << " to " << x1 << "," << y1 << " fc rgb 'red'\n";
	
		index++;  
	}*/

	/* dead space*/
	for (auto it = not_free_space.begin(); it != not_free_space.end(); ++it) {
		int x0 = (*it)->get_ll_x();
	    int y0 =  (*it)->get_ll_y();
		int x1 = (*it)->get_ur_x();
		int y1 = (*it)->get_ur_y();
		int midX = (x0+x1)/2;
		int midY = (y0+y1)/2;
		outgraph << "set object " << index << " rect from " 
		  	<< x0 << "," << y0 << " to " << x1 << "," << y1 << " fc rgb 'red'\n";
	
		index++;  
	}

	fstream outline("line", ios::out);

	/* draw boundary*/
	/*for (size_t i = 0; i < parser.dieArea.size(); ++i) {
		if (i < parser.dieArea.size() - 1) {
			outline << parser.dieArea[i].first << " " << parser.dieArea[i].second << endl;
			outline << parser.dieArea[i + 1].first << " " << parser.dieArea[i + 1].second << endl;
		} else {
			outline << parser.dieArea[i].first << " " << parser.dieArea[i].second << endl;
			outline << parser.dieArea[0].first << " " << parser.dieArea[0].second << endl;
		}
	}*/
	for (vector<Bound*>::iterator i = bound.begin(); i != bound.end(); i++) {
		if ((*i)->dir == H) {
			outline << (*i)->start << " " << (*i)->pos << endl;
			outline << (*i)->end << " " << (*i)->pos << endl << endl;
		}else{
			outline << (*i)->pos << " " << (*i)->start << endl;
			outline << (*i)->pos << " " << (*i)->end << endl << endl;
		}
	}


	outgraph << "set style line 1 lc rgb \"red\" lw 3\n";
	outgraph << "set border ls 1\n";
	outgraph << "set terminal png\n";
	outgraph << "set output \"graph.png\"\n";
	outgraph << "plot [" << parser.minOfWidth - 0.1 * parser.minOfWidth << ":" << parser.maxOfWidth*1.2 << "][" 
	<< parser.minOfHeight << ":" << parser.maxOfHeight*1.2 << "]\'line\' w l lt 2 lw 1\n";
	outgraph << "set terminal x11 persist\n";
	outgraph << "replot\n";
	outgraph << "exit";
	outgraph.close();

	int x = system("gnuplot output.gp");
}
