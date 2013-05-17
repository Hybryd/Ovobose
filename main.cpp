#include "Calibration.h"
#include "Scan.h"

int main(int argc, char ** argv )
{
//  Calibration cal("parameters.xml");
//  cal.circularPattern(10,5.6);
//  cal.save("matM","vecN");
  
  Scan scan("parameters.xml","out.gp",200);
  scan.read("matM","vecN");
  scan.launch();
  scan.save();
  
  return 0;
}
