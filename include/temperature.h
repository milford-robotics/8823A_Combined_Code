int graphTemp(vex::motor gmotor);

int warn(const char * warnMsg);

extern const char * motorIndexesShort[23];

extern const char * autonSelection;

void switchScreen();

void setMotorNames();

void drawAllUi();

void motorInfoScreen(vex::motor selectedMotor);

void displayMotor(vex::motor selectedMotor,int x,int y);

extern int screen;

extern int var;

void motorReassignScreen(vex::motor selectedMotor);

bool isMotorReversed(vex::motor motor);

vex::gearSetting getMotorCartridge(vex::motor motor);
