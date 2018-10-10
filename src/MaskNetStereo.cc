#include "MaskNet.h"
#include <iostream>
#include <fstream>
#include <iomanip>

namespace DynaSLAM
{

//#define U_SEGSt(a)\
		gettimeofday(&tvsv,0);\
		a = tvsv.tv_sec + tvsv.tv_usec/1000000.0
//struct timeval tvsv;
//double t1sv, t2sv,t0sv,t3sv;
//void tic_initsv(){U_SEGSt(t0sv);}
//void toc_finalsv(double &time){U_SEGSt(t3sv); time =  (t3sv- t0sv)/1;}
//void ticsv(){U_SEGSt(t1sv);}
//void tocsv(){U_SEGSt(t2sv);}
// std::cout << (t2sv - t1sv)/1 << std::endl;}

SegmentDynObject::SegmentDynObject(){
	std::cout << "Importing Mask R-CNN Settings..." << std::endl;
	ImportSettings();
	std::string x;   
	setenv("PYTHONPATH", this->py_path.c_str(), 1);
	x = getenv("PYTHONPATH");
    	Py_Initialize();
    this->cvt = new NDArrayConverter();
    //std::cout << "Module: "<< this->module_name.c_str() << std::endl;
    this->py_module = PyImport_ImportModule(this->module_name.c_str());
    //std::cout << this->py_module << std::endl;
    assert(this->py_module != NULL);
    //std::cout << "Class: "<< this->class_name.c_str() << " " << this->py_module << std::endl;
	this->py_class = PyObject_GetAttrString(this->py_module, this->class_name.c_str());
    assert(this->py_class != NULL);
    //std::cout << "Creating net instance..." << this->py_class << std::endl;
    this->net = PyInstance_New(this->py_class, NULL, NULL);
    //std::cout << "Hola" << std::endl;
    assert(this->net != NULL);
    //std::cout << "Creating net instance..." << std::endl;
//    cv::Mat image  = cv::Mat::zeros(480,640,CV_8UC3); //Be careful with size!!
//	  std::cout << "Loading net parameters..." << std::endl;
//    GetSegmentation(image);
}

/*SingleViewDepthEstimator::~SingleViewDepthEstimator(){
	delete this->py_module;
	delete this->py_class;
	delete this->net;
	delete this->cvt;
}*/

cv::Mat SegmentDynObjectStereo::GetSegmentation(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &MaskRight){
    cv::Mat image;
    cv::hconcat(image1,image2,image);
    PyObject* py_image = cvt->toNDArray(image.clone());
    assert(py_image != NULL);
    PyObject* py_mask_image = PyObject_CallMethod(this->net, const_cast<char*>(this->get_dyn_seg.c_str()),"(O)",py_image);
    cv::Mat seg = cvt->toMat(py_mask_image).clone();
    cv::Mat MaskLeft = seg.colRange(0,1241);
    MaskRight = seg.colRange(1241,2482);
    return MaskLeft;
}

void SegmentDynObjectStereo::ImportSettings(){
    std::string strSettingsFile = "/home/berta/Documents/ORB1/Examples/Stereo/MaskSettingsStereo.yaml";
    //std::string strSettingsFile = "/home/berta/Documents/ORB1/Examples/RGB-D/MaskSettings.yaml";
    cv::FileStorage fs(strSettingsFile.c_str(), cv::FileStorage::READ);
	fs["py_path"] >> this->py_path;
	fs["module_name"] >> this->module_name;
	fs["class_name"] >> this->class_name;
    fs["get_dyn_seg"] >> this->get_dyn_seg;

    std::cout << "    py_path: "<< this->py_path << std::endl;
    std::cout << "    module_name: "<< this->module_name << std::endl;
    std::cout << "    class_name: "<< this->class_name << std::endl;
    std::cout << "    get_dyn_seg: "<< this->get_dyn_seg << std::endl;
}


}






















