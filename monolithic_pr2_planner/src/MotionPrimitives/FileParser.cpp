#include <monolithic_pr2_planner/MotionPrimitives/FileParser.h>
#include <monolithic_pr2_planner/Constants.h>
#include <sbpl/headers.h>
#include <boost/shared_ptr.hpp>
#include <fstream>

using namespace monolithic_pr2_planner;
using namespace boost;
using namespace std;

void MotionPrimitiveFileParser::getNextLine(ifstream& file, stringstream& ss, string& line){
    getline(file, line);
    ss.str(line);
    ss.clear();
}

/*! \brief Parses a very specific arm motion primitives file. This is a very
 * brittle function because it needs exactly the right format.
 */
bool MotionPrimitiveFileParser::parseArmMotionPrimitives(string filename,
                                                         std::vector<MotionPrimitivePtr>& prims){
    ifstream file;
    file.open(filename);
    string line;
    string label;
    double dvalue;
    int ivalue;
    stringstream ss;

    // skip to the primitives
    for (int i=0; i < 7; i++){
        getNextLine(file, ss, line);
    }
    while (!file.eof()){
        ArmMotionPrimitivePtr mprim = make_shared<ArmMotionPrimitive>();
        // gets groups
        ss >> label >> ivalue;
        mprim->setGroup(ivalue);

        // gets id
        getNextLine(file, ss, line);
        ss >> label >> ivalue;
        mprim->setID(ivalue);
         
        // gets coords
        getNextLine(file, ss, line);
        ss >> label;
        vector<int> coord;
        while (ss >> ivalue){
            coord.push_back(ivalue);
        }
        GraphStateMotion motion(GRAPH_STATE_SIZE,0);
        motion[GraphStateElement::OBJ_X] = coord[0];
        motion[GraphStateElement::OBJ_Y] = coord[1];
        motion[GraphStateElement::OBJ_Z] = coord[2];
        motion[GraphStateElement::OBJ_ROLL] = coord[3];
        motion[GraphStateElement::OBJ_PITCH] = coord[4];
        motion[GraphStateElement::OBJ_YAW] = coord[5];
        motion[GraphStateElement::R_FA] = coord[6];
        motion[GraphStateElement::L_FA] = coord[7];

        mprim->setEndCoord(motion);

        // gets num interm steps
        int num_interm_steps;
        getNextLine(file, ss, line);
        ss >> label >> num_interm_steps;
        IntermSteps interm_steps;
        std::vector<double> init_step(GRAPH_STATE_SIZE,0);
        interm_steps.push_back(init_step);
        for (int i=0; i < num_interm_steps; i++){
            vector<double> step;
            getNextLine(file, ss, line);
            while (ss >> dvalue){
                step.push_back(dvalue);
            }
            std::vector<double> g_step(GRAPH_STATE_SIZE,0);
            g_step[GraphStateElement::OBJ_X] = step[0];
            g_step[GraphStateElement::OBJ_Y] = step[1];
            g_step[GraphStateElement::OBJ_Z] = step[2];
            g_step[GraphStateElement::OBJ_ROLL] = step[3];
            g_step[GraphStateElement::OBJ_PITCH] = step[4];
            g_step[GraphStateElement::OBJ_YAW] = step[5];
            g_step[GraphStateElement::R_FA] = step[6];
            g_step[GraphStateElement::L_FA] = step[7];

            interm_steps.push_back(g_step);
        }
        mprim->setIntermSteps(interm_steps);
        // skip the last few lines
        for (int i=0; i < 3; i++){
            getNextLine(file, ss, line);
        }
        prims.push_back(mprim);
    }
    return true;
}

// TODO: do verification as seen in the old code?
bool MotionPrimitiveFileParser::parseBaseMotionPrimitives(string filename,
                                                          std::vector<MotionPrimitivePtr>& prims){
    ifstream file;
    file.open(filename.c_str());
    string line;
    string label;
    double dvalue;
    int ivalue;
    stringstream ss;

    for (int i=0; i < 3; i++){
        getNextLine(file, ss, line);
    }


    getNextLine(file, ss, line);
    while (!file.eof()){
        // id
        BaseMotionPrimitivePtr mprim = make_shared<BaseMotionPrimitive>();
        ss >> label >> ivalue;
        mprim->setID(ivalue);

        // start angle
        getNextLine(file, ss, line);
        ss >> label >> ivalue;
        mprim->start_angle(ivalue);

        // end pose
        getNextLine(file, ss, line);
        ss >> label;
        vector<int> coord;
        while (ss >> ivalue){
            coord.push_back(ivalue);
        }
        GraphStateMotion motion(GRAPH_STATE_SIZE,0);
        motion[GraphStateElement::BASE_X] = coord[0];
        motion[GraphStateElement::BASE_Y] = coord[1];

        // since these mprims are specific to each theta, we need to adjust this
        // to be relative to the start angle. for example, if our base is
        // currently at theta = 2 and coord[2] = 3 (the final angle after this
        // motion primitive), we actually want our motion to be 1, not 3. 
        // TODO could be a bug here if coord[2] = 0, start_angle = 15
        motion[GraphStateElement::BASE_THETA] = coord[2] - mprim->start_angle();

        mprim->setEndCoord(motion);
        
        // cost
        getNextLine(file, ss, line);
        ss >> label >> ivalue;
        mprim->setAdditionalCostMult(ivalue);
       
        // gets num interm steps
        int num_interm_steps;
        getNextLine(file, ss, line);
        IntermSteps interm_steps;
        ss >> label >> num_interm_steps;

        for (int i=0; i < num_interm_steps; i++){
            getNextLine(file, ss, line);
            vector<double> step;
            while (ss >> dvalue){
                step.push_back(dvalue);
            }
            std::vector<double> g_step(GRAPH_STATE_SIZE,0);
            g_step[GraphStateElement::BASE_X] = step[0];
            g_step[GraphStateElement::BASE_Y] = step[1];
            g_step[GraphStateElement::BASE_THETA] = step[2];

            interm_steps.push_back(g_step);
        }
        mprim->setIntermSteps(interm_steps);
        prims.push_back(mprim);
        getNextLine(file, ss, line);
    }
    return true;
}
