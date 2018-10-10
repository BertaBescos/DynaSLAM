#ifndef __MASKNETSTEREO_H
#define __MASKNETSTEREO_H

#ifndef NULL
#define NULL   ((void *) 0)
#endif

#include <python2.7/Python.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <cstdio>
#include <boost/thread.hpp>
#include "include/Conversion.h"

namespace DynaSLAM
{

class SegmentDynObjectStereo{
private:
	NDArrayConverter *cvt; 	/*!< Converter to NumPy Array from cv::Mat */
	PyObject *py_module; 	/*!< Module of python where the Mask algorithm is implemented */
	PyObject *py_class; 	/*!< Class to be instanced */
	PyObject *net; 			/*!< Instance of the class */
	std::string py_path; 	/*!< Path to be included to the environment variable PYTHONPATH */
	std::string module_name; /*!< Detailed description after the member */
	std::string class_name; /*!< Detailed description after the member */
        std::string get_dyn_seg; 	/*!< Detailed description after the member */

	void ImportSettings();
public:

        SegmentDynObjectStereo();

        ~SegmentDynObjectStereo(){
		delete this->py_module;
		delete this->py_class;
		delete this->net;
		delete this->cvt;
	};
        cv::Mat GetSegmentation(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &MaskRight);
};


}

#endif
