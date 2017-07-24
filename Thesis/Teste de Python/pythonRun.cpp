#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <boost/array.hpp>

//namespace declaration
using namespace std;
using namespace cv;
using namespace boost;

#include <python2.7/Python.h>
#include <string>
#include <sstream>

int main(){
    char *p = (char*)"myPythonProgram";
    PyObject *morphsnakes, *np;

    Py_SetProgramName(p);
    Py_Initialize();
    //Init import modules
    _PyImport_Init();
    //Add current path to sys.path
    PyRun_SimpleString("import sys\n" "sys.path.append('.')\n");
    //Import modules
    np = PyImport_ImportModule("numpy");
    morphsnakes = PyImport_ImportModule("morphsnakes");

    //Mat src = imread("./testimages/mama07ORI.bmp");

    //Exec python script
    PyRun_SimpleString("exec(open('cpptest.py').read())");
    _PyImport_Fini();
    Py_Finalize();
  }
