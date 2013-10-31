#include <monolithic_pr2_planner/MotionPrimitives/FileParser.h>
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

// this is a very brittle function
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
        mprim->setEndCoord(coord);

        // gets num interm steps
        int num_interm_steps;
        getNextLine(file, ss, line);
        ss >> label >> num_interm_steps;
        IntermSteps interm_steps;
        for (int i=0; i < num_interm_steps; i++){
            vector<double> step;
            getNextLine(file, ss, line);
            while (ss >> dvalue){
                step.push_back(dvalue);
            }
            interm_steps.push_back(step);
        }
        mprim->setIntermSteps(interm_steps);
        // skip the last few lines
        for (int i=0; i < 3; i++){
            getNextLine(file, ss, line);
        }
        ROS_DEBUG_NAMED(CONFIG_LOG, "arm mprim created:");
        mprim->print();
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
        mprim->setStartAngle(ivalue);

        // end pose
        getNextLine(file, ss, line);
        ss >> label;
        vector<int> coord;
        while (ss >> ivalue){
            coord.push_back(ivalue);
        }
        mprim->setEndCoord(coord);
        
        // cost
        getNextLine(file, ss, line);
        ss >> label >> ivalue;
        mprim->setCost(ivalue);
       
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
            interm_steps.push_back(step);
        }
        ROS_DEBUG_NAMED(CONFIG_LOG, "base motion primitive created:");
        mprim->setIntermSteps(interm_steps);
        mprim->print();
        prims.push_back(mprim);
        getNextLine(file, ss, line);
    }
    return true;
}
