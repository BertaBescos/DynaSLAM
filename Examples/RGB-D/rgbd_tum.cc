/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>
#include<opencv2/core/core.hpp>

#include "Geometry.h"
#include "MaskNet.h"
#include <System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 5 && argc != 6 && argc != 7)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association (path_to_masks) (path_to_output)" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    std::cout << "nImages: " << nImages << std::endl;

    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Initialize Mask R-CNN
    cout << "Loading Mask R-CNN. This could take a while..." << endl;
    ORBmask::SegmentDynObject *MaskNet;
    if (argc==6)
    {
        MaskNet = new ORBmask::SegmentDynObject();
    }
    cout << "Mask R-CNN loaded!" << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    // Vector for light tracking time statistics
    vector<float> vTimesLightTrack;

    // Vector for Full Dynamic Object Detection time statistics
    vector<float> vTimesFDOD;

    // Vector for Background Reconstruction time statistics
    vector<float> vTimesBR;

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Dilation settings
    int dilation_size = 15;
    cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,
                                           cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                           cv::Point( dilation_size, dilation_size ) );

    // Main loop
    cv::Mat imRGB, imD;
    cv::Mat imRGBOut, imDOut,maskOut;

    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);

        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Segment out the images
        cv::Mat mask = cv::Mat::ones(480,640,CV_8U);
        if(argc == 6 || argc == 7){
            cv::Mat maskRCNN;
            maskRCNN = MaskNet->GetSegmentation(imRGB,string(argv[5]),vstrImageFilenamesRGB[ni].replace(0,4,""));
            cv::Mat maskRCNNdil = maskRCNN.clone();
            cv::dilate(maskRCNN,maskRCNNdil, kernel);
            mask = mask - maskRCNNdil;
        }

        if(argc ==5)
        {
            mask = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni].replace(0,4,"mask/"),CV_LOAD_IMAGE_UNCHANGED);
            if (mask.empty()) {mask = cv::Mat::ones(480,640,CV_8U);}
         }

        // Pass the image to the SLAM system
        if (argc == 7)
        {
            SLAM.TrackRGBD(imRGB,imD,mask,tframe,vTimesLightTrack,vTimesFDOD,vTimesBR,imRGBOut,imDOut,maskOut);
        }
        else
        {
            SLAM.TrackRGBD(imRGB,imD,mask,tframe,vTimesLightTrack,vTimesFDOD,vTimesBR);
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        if (argc == 7)
        {
            std::cout << string(argv[6]) + "/rgb/" + vstrImageFilenamesRGB[ni] << std::endl;
            cv::imwrite(string(argv[6]) + "/rgb/" + vstrImageFilenamesRGB[ni],imRGBOut);
            cv::imwrite(string(argv[6]) + "/" + vstrImageFilenamesD[ni],imDOut);
            cv::imwrite(string(argv[6]) + "/mask/" + vstrImageFilenamesRGB[ni],maskOut);
        }

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // LightTrack time statistics
    sort(vTimesLightTrack.begin(),vTimesLightTrack.end());
    float totalTimeLightTrack = 0;
    for(int ni=0; ni<(int)vTimesLightTrack.size(); ni++)
    {
        totalTimeLightTrack += vTimesLightTrack[ni];
    }
    cout << "------- Light Track " << endl;
    cout << "median tracking time: " << vTimesLightTrack[((int)vTimesLightTrack.size())/2] << endl;
    cout << "mean tracking time: " << totalTimeLightTrack/((int)vTimesLightTrack.size()) << endl;

    // Full Dynamic Objects Detection time statistics
    sort(vTimesFDOD.begin(),vTimesFDOD.end());
    float totalTimeFDOD = 0;
    for(int ni=0; ni<(int)vTimesFDOD.size(); ni++)
    {
        totalTimeFDOD += vTimesFDOD[ni];
    }
    cout << "------- Full Dynamic Objects Detection " << endl;
    cout << "median tracking time: " << vTimesFDOD[((int)vTimesFDOD.size())/2] << endl;
    cout << "mean tracking time: " << totalTimeFDOD/((int)vTimesFDOD.size()) << endl;

    // Background Reconstruction time statistics
    sort(vTimesBR.begin(),vTimesBR.end());
    float totalTimeBR = 0;
    for(int ni=0; ni<(int)vTimesBR.size(); ni++)
    {
        totalTimeBR += vTimesBR[ni];
    }
    cout << "------- Background Reconstruction " << endl;
    cout << "median tracking time: " << vTimesBR[((int)vTimesBR.size())/2] << endl;
    cout << "mean tracking time: " << totalTimeBR/((int)vTimesBR.size()) << endl;


    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
