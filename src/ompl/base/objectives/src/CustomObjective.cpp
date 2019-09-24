/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Luis G. Torres */

#include "ompl/base/objectives/CustomObjective.h"
#include "ompl/tools/config/MagicConstants.h"
#include <fstream>
#include <iostream>
#include <limits>
#include <python2.7/Python.h>
#include <stdlib.h>
#include <string>

using namespace std;

PyObject *pName, *pModule, *pDict, *pFunc;

ompl::base::CustomObjective::
CustomObjective(const SpaceInformationPtr &si) :
    MinimaxObjective(si)
{
    this->setCostThreshold(Cost(std::numeric_limits<double>::infinity()));
    this->setPython();
}

void ompl::base::CustomObjective::setPython()
{
    string directory, file, function, line;
    const string configFilePath = "/home/jgkawell/ws_moveit/config/planning.cfg";
    ifstream configFile (configFilePath);
    if (configFile.is_open())
    {
        int lineNum = 0;
        while (getline(configFile, line))
        {
            switch (lineNum)
            {
                case 0:
                    // Skip first line since it's just a comment
                    break;
                case 1:
                    directory = line;
                    OMPL_INFORM("Python Directory: %s", directory.c_str());
                    break;
                case 2:
                    file = line;
                    OMPL_INFORM("Python File Name: %s", file.c_str());
                    break;
                case 3:
                    function = line;
                    OMPL_INFORM("Python Function Name: %s", function.c_str());
                    break;
                default:
                    OMPL_WARN("Not a valid configuration line: %d", lineNum);
            }
            lineNum += 1;
        }
        configFile.close();

        // Set PYTHONPATH TO working directory
        setenv("PYTHONPATH",".",1);
        Py_Initialize();
        PyObject* sysPath = PySys_GetObject((char*)"path");
        PyList_Append(sysPath, PyString_FromString(directory.c_str()));

        // Reference the python module to call
        pName = PyString_FromString((char*)file.c_str());
        pModule = PyImport_Import(pName);
        pDict = PyModule_GetDict(pModule);
        pFunc = PyDict_GetItemString(pDict, (char*)function.c_str());
    }
    else
    {
        OMPL_ERROR("Unable to open Python config file at path: %s", configFilePath); 
    }
}

ompl::base::Cost ompl::base::CustomObjective::stateCost(const State *s) const
{
    double cResult;

    try
    {
        // Try making python call
        if (PyCallable_Check(pFunc))
        {
            // Get cost from Python module
            PyObject *pResult = PyObject_CallObject(pFunc, 0);

            // Convert Python float to C++ double
            cResult = PyFloat_AsDouble(pResult);
            if (PyErr_Occurred())
            {
                OMPL_ERROR("Something broke in Python to C++ conversion!");
            }
            
            // Cleanup
            Py_DECREF(pResult);
        }
        else 
        {
            PyErr_Print();
        }

    }
    catch (int e)
    {
        OMPL_ERROR("An exception occurred: %d ", e);
        return Cost(si_->getStateValidityChecker()->clearance(s));
    }

    return Cost(cResult);
}

bool ompl::base::CustomObjective::isCostBetterThan(Cost c1, Cost c2) const
{
    return c1.value() > c2.value();
}

ompl::base::Cost ompl::base::CustomObjective::identityCost() const
{
    return Cost(std::numeric_limits<double>::infinity());
}

ompl::base::Cost ompl::base::CustomObjective::infiniteCost() const
{
    return Cost(-std::numeric_limits<double>::infinity());
}
