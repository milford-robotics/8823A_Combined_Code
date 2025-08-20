#include <fstream>
#include <iostream>
#include <vector>



int recordTo(std::string fileName, std::vector<vex::motor> motors);
int dumpSD(std::string fileName);
int playBack(std::string fileName,std::vector<vex::motor> motors);
extern bool endRecord;