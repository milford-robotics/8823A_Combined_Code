#include "vex.h"


bool endRecord=false;
int smplrt=3;

int dumpSD(std::string fileName){
    
    std::ifstream inFile;

    inFile.open(fileName);
    
    if(inFile.fail()){
        std::cout << "Tried to dump file, but it didn't exist" << std::endl;
        return 1;
    }

    std::string fileText;


    while(true){

        std::getline(inFile,fileText);

        if(inFile.eof()){
            inFile.close();
            return 0;
        }
        else{
            std::cout << fileText <<std::endl;
        }
    }
}

int playBack(std::string fileName,std::vector<vex::motor> motors){
    std::ifstream inFile;
    inFile.open(fileName);
    if(inFile.fail()){
        std::cout << "Tried to playback file but it was invalid" << std::endl;
        return 1;
    }

    float num=0;
    while(true){
        
        for(vex::motor Motor:motors){
                    
            inFile >> num;

            if(inFile.eof()){
                return 0;
            }
            else{
                Motor.spin(vex::forward,num,vex::volt);
            }
        }
        vex::task::sleep(smplrt);
    }
    
    return 0;
}


//function to erase the data on a file
//if there is no file at the path, create an empty file there
void eraseSD(std::string fileName){
    std::ofstream outFile;
    outFile.open(fileName);

    outFile.close();
}

int recordTo(std::string fileName, std::vector<vex::motor> motors){
    std::ofstream outFile;
    outFile.open(fileName);
    if(outFile.fail()){
        printf("Tried to record to file, but it didn't exist \n");
        return 1;
    }
    
    while(!endRecord){
        // outFile.open(fileName, std::ios_base::app );
        for(vex::motor Motor:motors){

            float command=Motor.voltage();

            outFile << command << " ";
        }

        outFile << std::endl;

        // outFile.close();
        vex::task::sleep(smplrt);
    }
    outFile.close();
    return 0;
}
