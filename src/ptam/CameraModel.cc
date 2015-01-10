#include <gvars3/instances.h>
#include <iostream>
#include "CameraModel.h"
#include "ATANCamera.h"
#include "PolynomialCamera.h"

using namespace GVars3;
using namespace std;
using namespace ptam;

bool CameraModel::firstCreate = true;
bool CameraModel::firstCreatesec = true;
bool CameraModel::polynomial = false;
bool CameraModel::polynomialsec = false;
auto_ptr<CameraModel> CameraModel::cameraPrototype;
auto_ptr<CameraModel> CameraModel::cameraPrototypesec;

CameraModel::CameraModel() {
    mvImageSize[0] = mvImageSize[1] = 0;
}

CameraModel* CameraModel::CreateCamera(int camnum) {
    if((firstCreate&&!camnum)) {
        // This method might be called a lot, so we only check
        // the configuration on the first time
        string calibType;
        string calibFile;
        if (!camnum){
            firstCreate = false;
            calibType = GV3::get<string>("Camera.Type", "Polynomial", /*HIDDEN*/ SILENT);
            calibFile = GV3::get<string>("Camera.File", "", /*HIDDEN*/ SILENT);

            if (calibType == "Polynomial") {
                // This is a polynomial camera
                cameraPrototype.reset(new PolynomialCamera(calibFile.c_str()));
                polynomial = true;
            } else if (calibType == "ATAN") {
                // This is a ATAN camera
                GUI.LoadFile(calibFile);
                Vector<NUMTRACKERCAMPARAMETERS> vTest = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, HIDDEN);
                if(vTest == ATANCamera::mvDefaultParams) {
                    cerr << "! Camera.Parameters is not set, need to run the CameraCalibrator tool" << endl;
                    cerr << "  and/or put the Camera.Parameters= line into the appropriate .cfg file." << endl;
                    exit(1);
                }
                cameraPrototype.reset(new ATANCamera("Camera"));
                polynomial = false;
            }
        }
        else if (camnum){
            int adcamIndex = camnum - 1;
            firstCreatesec = false;
            string camType = "Camerasec" + adcamIndex + ".Type";
            string camFile = "Camerasec" + adcamIndex + ".File";
            calibType = GV3::get<string>(camType, "Polynomial", /*HIDDEN*/ SILENT);
            calibFile = GV3::get<string>(camFile, "", /*HIDDEN*/ SILENT);

            if (calibType == "Polynomial") {
                // This is a polynomial camera
                cameraPrototypesec.reset(new PolynomialCamera(calibFile.c_str()));
                polynomialsec = true;
            } else if (calibType == "ATAN") {
                // This is a ATAN camera
                GUI.LoadFile(calibFile);
                Vector<NUMTRACKERCAMPARAMETERS> vTest = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, HIDDEN);
                if(vTest == ATANCamera::mvDefaultParams) {
                    cerr << "! Camera.Parameters is not set, need to run the CameraCalibrator tool" << endl;
                    cerr << "  and/or put the Camera.Parameters= line into the appropriate .cfg file." << endl;
                    exit(1);
                }
                cameraPrototypesec.reset(new ATANCamera("Camera"));
                polynomialsec = false;
            }
        }
    }

    if (!camnum){
        if(polynomial)// TODO: currently, dual cameras should be with the same type
            return new PolynomialCamera(*static_cast<PolynomialCamera*>(cameraPrototype.get()));
        else return new ATANCamera(*static_cast<ATANCamera*>(cameraPrototype.get()));
    }
    else{
        if(polynomialsec)// TODO: currently, dual cameras should be with the same type
            return new PolynomialCamera(*static_cast<PolynomialCamera*>(cameraPrototypesec.get()));
        else return new ATANCamera(*static_cast<ATANCamera*>(cameraPrototypesec.get()));
    }
}
