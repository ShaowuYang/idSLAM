// Copyright 2008 Isis Innovation Limited
#include "OpenGL.h"
#include <gvars3/instances.h>
#include "CameraCalibrator.h"
#include <TooN/SVD.h>
#include <fstream>
#include <stdlib.h>

using namespace CVD;
using namespace std;
using namespace GVars3;

int main()
{
  cout << "  Welcome to CameraCalibrator " << endl;
  cout << "  -------------------------------------- " << endl;
  cout << "  Parallel tracking and mapping for Small AR workspaces" << endl;
  cout << "  Copyright (C) Isis Innovation Limited 2008 " << endl;
  cout << endl;  
  cout << "  Parsing calibrator_settings.cfg ...." << endl;
  
  GUI.LoadFile("calibrator_settings.cfg");

  GUI.StartParserThread();
  atexit(GUI.StopParserThread); // Clean up readline when program quits
  
  GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, SILENT);

  try
    {
      CameraCalibrator c;
      c.Run();
    }
  catch(CVD::Exceptions::All e)
    {
      cout << endl;
      cout << "!! Failed to run CameraCalibrator; got exception. " << endl;
      cout << "   Exception was: " << endl;
      cout << e.what << endl;
    }
}

