#include <monolithic_pr2_planner/MotionPrimitives.h>

using namespace monolithic_pr2_planner;

bool MotionPrimitivesFileParser::verifyFileFormat(FILE* mprim_file){
}

bool MotionPrimitivesFileParser::getNextToken(FILE* mprim_file){
    char token[1024]
    if (fscanf(mprim_file, "%s", token) < 1){

}
