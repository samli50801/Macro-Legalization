#include <ctime>
#include "Force_Directed.h"
#include "FreeSpace.h"
#include "preprocessor.h"
#include "tile.h"
#include "legalizer.h"
#include "cluster.h"
#include "CG.h"
#include "bufferLegalizer.h"
#include <time.h>

Parser parser;
map<string, Macro*> macro;			// All Macro
vector<Component*> comp;			// All component
unordered_set<Component*> free_space;		// All the space which can place standard cell
unordered_set<Component*> not_free_space;	// All the space which can't place standard cell
vector<Component*> boundaryComp;
vector<Component*> illegal;
vector<Bound*> bound;				// Segments of the design boundary
void plot(Rectangle);										
vector<vector<Component*>> overlapGroup;	// overlapped group for corner-stitching
vector<bc::Group*> groups;			// clustering groups
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

	plot(preprocessor.getBoundingRect());
	
	// For corner-stitching
	Force_Directed fd(parser._mcsbmc * parser.dbuPerMicron);
	fd.move_macro_inside_boundary();	// Move macros ounside the design into the nearest boundary
	start = clock();
	fd.do_forceDirected();
	end = clock();
	duration_sec = double(end-start) / CLOCKS_PER_SEC;
	Log << "Force-directed: " << duration_sec << "sec" << endl;

	plot(preprocessor.getBoundingRect());

	cg::CG cg(comp, boundaryComp, parser);
	cg.legalize();
	cg.optBufferArea();
	deletePoint = cg.deletedPoint;

    	Legalizer legalizer(comp,
        preprocessor.getPlane(), 
        preprocessor.getBoundingRect(),
        parser);

	start = clock();
	legalizer.legalize(deletePoint);
	end = clock();
	duration_sec = double(end-start) / CLOCKS_PER_SEC;
	Log << "corner-stitch: " << duration_sec << "sec" << endl;
	plot(preprocessor.getBoundingRect());

	buffer::BufferLegalizer bufferLegalizer(bound, comp, boundaryComp, parser);
	bufferLegalizer.legalize();
	illegal = bufferLegalizer.getIllegal();
	free_space = bufferLegalizer.getFreeSpace();
	not_free_space = bufferLegalizer.getDeadSpace();
	cout << "illegal: " << illegal.size() << endl;
	plot(preprocessor.getBoundingRect());

	preprocessor.resetPlane();
	legalizer.needCheckBuffer();
	legalizer.buildBufferPlane(free_space, preprocessor.getBufferPlane());
	legalizer.setPlane(preprocessor.getPlane());
	legalizer.legalize(deletePoint);
	cout << "comp size: " << comp.size() << endl;

	/* gnuplot */
	plot(preprocessor.getBoundingRect());

	buffer::BufferLegalizer bufferLegalizer2(bound, comp, boundaryComp, parser);
	bufferLegalizer2.legalize();
	illegal = bufferLegalizer2.getIllegal();
    	if(illegal.empty())
		cout << "Successfully buffer legalize\n";
   	else
		cout << "Fail to legalize\n";
	
	/*check if the final placement is legal (no overlappings) */
	fd.generate_overlap_group();		// Generate overlapped group
	overlapGroup = fd.getGroup();
	if (overlapGroup.empty())
		cout << "Successfully legalize\n";
	else 
		cout << "Fail to legalize\n";

	parser.recoverCoordinateSystem(comp);
	parser.writeDef(output, comp);
    	parser.score_evaluation(comp, not_free_space);

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
	
 	for (vector<Component*>::iterator i = comp.begin(); i != comp.end(); i++) {
		string NodeName = (*i)->getName();
	    int x0 = (*i)->get_ll_x();
	    int y0 =  (*i)->get_ll_y();
		int x1 = (*i)->get_ur_x();
		int y1 = (*i)->get_ur_y();
		int midX = (x0+x1)/2;
		int midY = (y0+y1)/2;

		if (isInOverlapGroup(*i)){
			outgraph << "set object " << index << " rect from " 
		  		<< x0 << "," << y0 << " to " << x1 << "," << y1 << " fs empty border lc rgb '#ff0000'\n";
				//<< "set label " << "\"" << NodeName << "\"" << " at " << midX << "," << midY << " center " << "font \",8\"\n";
		}else {
			if ((*i)->getType() == FIXED) {
				outgraph << "set object " << index << " rect from " 
		  		<< x0 << "," << y0 << " to " << x1 << "," << y1 << " fs empty border lc rgb 'green'\n";
			} else {
				outgraph << "set object " << index << " rect from " 
		  		<< x0 << "," << y0 << " to " << x1 << "," << y1 << " fs empty border lc rgb '#000000'\n";
				//<< "set label " << "\"" << NodeName << "\"" << " at " << midX << "," << midY << " center " << "font \",8\"\n";
			}
		}
		
		index++;
	}
	
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
	for (auto it = boundaryComp.begin(); it != boundaryComp.end(); ++it) {
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
	}

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

	for (auto it = illegal.begin(); it != illegal.end(); ++it) {
		int x0 = (*it)->get_ll_x();
	    int y0 =  (*it)->get_ll_y();
		int x1 = (*it)->get_ur_x();
		int y1 = (*it)->get_ur_y();
		int midX = (x0+x1)/2;
		int midY = (y0+y1)/2;
		outgraph << "set object " << index << " rect from " 
		  	<< x0 << "," << y0 << " to " << x1 << "," << y1 << " fc rgb 'purple'\n";
	
		index++;  
	}

	fstream outline("line", ios::out);

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
