#include "Calibration.h"


int main(int argc, char ** argv )
{
  Calibration cal("parameters.xml");
  cal.circularPattern(10,5.6);
  cal.save("matM","vecN");
  return 0;
}
