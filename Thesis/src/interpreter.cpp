#include <python2.7/Python.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>

int main()
{
    PyObject *pName, *pModule, *pDict, *pClass, *pInstance, *pValue;

    // Initialize the Python interpreter
    Py_Initialize();

    // Build the name object
    pName = PyString_FromString("Adder");
    // Load the module object
    pModule = PyImport_Import(pName);
    // pDict is a borrowed reference
    pDict = PyModule_GetDict(pModule);
    // Build the name of a callable class
    pClass = PyDict_GetItemString(pDict, "Adder");
    // Create an instance of the class
    if (PyCallable_Check(pClass))
    {
        pInstance = PyObject_CallObject(pClass,NULL);
    }
    else
    {
        std::cout << "Cannot instantiate the Python class" << std::endl;
    }

    int sum = 0;
    int x;

    char * add2s = {"add2"};
    char * is = {"(i)"};

    for (size_t i = 0 ; i < 5 ; i++)
    {
        x = rand() % 100;
        sum += x;
        PyObject_CallMethod(pInstance, add2s, is,x);
        if(pValue)
          Py_DECREF(pValue);
        else
          PyErr_Print();
    }
    PyObject_CallMethod(pInstance, "printSum", NULL);
    std::cout << "the sum via C++ is " << sum << std::endl;

    std::getchar();
    Py_Finalize();
}
