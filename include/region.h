/*
 *  Regionprops
 *  Copyright 2015 Andrea Pennisi
 *
 *  This file is part of Regionprops and it is distributed under the terms of the
 *  GNU Lesser General Public License (Lesser GPL)
 *
 *
 *
 *  Regionprops is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Regionprops is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with Regionprops.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  Regionprops has been written by Andrea Pennisi
 *
 *  Please, report suggestions/comments/bugs to
 *  andrea.pennisi@gmail.com
 *
 */

#ifndef REGION_H
#define REGION_H

#include <opencv2/opencv.hpp>
#include <iostream>

class Region {
    public:
        /**
         * @brief Region new Object
         */
        Region() {}
        /**
         * @brief Area
         * @return return area
         */
        inline double Area() const
        {
            return area;
        }
        /**
         * @brief setArea set area to _area
         * @param _area
         */
        inline void setArea(const double &_area)
        {
            area = _area;
        }
        /**
         * @brief Perimeter
         * @return return the perimeter
         */
        inline double Perimeter() const
        {
            return perimeter;
        }
        /**
         * @brief setPerimeter set perimeter to _perimeter
         * @param _perimeter
         */
        inline void setPerimeter(const double &_perimeter)
        {
            perimeter = _perimeter;
        }
        /**
         * @brief Moments return moments;
         * @param _moments
         * @return moments
         */
        inline cv::Moments Moments() const
        {
            return moments;
        }
        /**
         * @brief setMoments set moments to _moments
         * @param _moments
         */
        inline void setMoments(const cv::Moments &_moments)
        {
            moments = _moments;
        }
        /**
         * @brief BoundingBox
         * @return the boundingBox
         */
        inline cv::Rect BoundingBox() const
        {
            return boundingBox;
        }
        /**
         * @brief setBoundingBox set boundingBox to _boundingBox
         * @param _boundingBox
         */
        inline void setBoundingBox(const cv::Rect &_boundingBox)
        {
            boundingBox = _boundingBox;
        }
        /**
         * @brief ConvexHull
         * @return return the convex hull
         */
        inline std::vector<cv::Point> ConvexHull() const
        {
            return convex_hull;
        }
        /**
         * @brief setConvexHull set convex_hull to _convex_hull
         * @param _convex_hull
         */
        inline void setConvexHull(const std::vector<cv::Point> &_convex_hull)
        {
            convex_hull = _convex_hull;
        }
        /**
         * @brief ConvexArea
         * @return the convex area
         */
        inline double ConvexArea() const
        {
            return convex_area;
        }
        /**
         * @brief setConvexArea set convex_area to _convex_area
         * @param _convex_area
         */
        inline void setConvexArea(const double &_convex_area)
        {
            convex_area = _convex_area;
        }
        /**
         * @brief Ellipse
         * @return the ellipse
         */
        inline cv::RotatedRect Ellipse() const
        {
            return ellipse;
        }
        /**
         * @brief setEllipse set the ellipse to _ellipse
         * @param _ellipse
         */
        inline void setEllipse(const cv::RotatedRect &_ellipse)
        {
            ellipse = _ellipse;
        }
        /**
         * @brief Orientation
         * @return the orientation of the ellipse
         */
        inline double Orientation() const
        {
            return orientation;
        }
        /**
         * @brief setOrientation set orientation to _orientation
         * @param _orientation
         */
        inline void setOrientation(const double &_orientation)
        {
            orientation = _orientation;
        }
        /**
         * @brief MinorAxis
         * @return the ellipse minor axis
         */
        inline double MinorAxis() const
        {
            return minoraxis_length;
        }
        /**
         * @brief setMinorAxis set minoraxis_length to minor_axis
         * @param minor_axis
         */
        inline void setMinorAxis(const double &minor_axis)
        {
            minoraxis_length = minor_axis;
        }
        /**
         * @brief MinorAxis
         * @return the ellipse minor axis
         */
        inline double MajorAxis() const
        {
            return majoraxis_length;
        }
        /**
         * @brief setMinorAxis set minoraxis_length to minor_axis
         * @param minor_axis
         */
        inline void setMajorAxis(const double &major_axis)
        {
            majoraxis_length = major_axis;
        }
        /**
         * @brief Approx
         * @return the approximate hull of the contour
         */
        inline std::vector<cv::Point> Approx() const
        {
            return approx;
        }
        /**
         * @brief setApprox set approx to _approx
         * @param _approx
         */
        inline void setApprox(const std::vector<cv::Point> &_approx)
        {
            approx = _approx;
        }
        /**
         * @brief FilledImage
         * @return the image where region is white and others are black
         */
        inline cv::Mat FilledImage() const
        {
            return filledImage;
        }
        /**
         * @brief setFilledImage set filledImage to _filledImage
         * @param _filledImage
         */
        inline void setFilledImage(const cv::Mat &_filledImage)
        {
            filledImage = _filledImage;
        }
        /**
         * @brief Centroid
         * @return the centroid of the hull
         */
        inline cv::Point Centroid() const
        {
            return centroid;
        }
        /**
         * @brief setCentroid set the centroid to _centroid
         * @param _centroid
         */
        inline void setCentroid(const cv::Point &_centroid)
        {
            centroid = _centroid;
        }
        /**
         * @brief AspectRatio
         * @return the aspect ratio of the hull
         */
        inline double AspectRatio() const
        {
            return aspect_ratio;
        }
        /**
         * @brief setAspectRatio set aspect_ratio to _aspect_ratio
         * @param _aspect_ratio
         */
        inline void setAspectRatio(const double &_aspect_ratio)
        {
            aspect_ratio = _aspect_ratio;
        }
        /**
         * @brief EquivalentDiameter
         * @return the equivalent diameter of the circle with same as area as that of region
         */
        inline double EquivalentDiameter() const
        {
            return equi_diameter;
        }
        /**
         * @brief setEquivalentDiameter set equi_diameter to _equi_diameter
         * @param _equi_diameter
         */
        inline void setEquivalentDiameter(const double &_equi_diameter)
        {
            equi_diameter = _equi_diameter;
        }
        /**
         * @brief Eccentricity
         * @return the eccentricity of the ellipse
         */
        inline double Eccentricity() const
        {
            return eccentricity;
        }
        /**
         * @brief setEccentricity set the eccentricity to _eccentricity
         * @param _eccentricity
         */
        inline void setEccentricity(const double &_eccentricity)
        {
            eccentricity = _eccentricity;
        }
        /**
         * @brief FilledArea
         * @return the number of white pixels in filledImage
         */
        inline double FilledArea() const
        {
            return filledArea;
        }
        /**
         * @brief setFilledArea set filledArea to _filledArea
         * @param _filledArea
         */
        inline void setFilledArea(const double &_filledArea)
        {
            filledArea = _filledArea;
        }
        /**
         * @brief PixelList
         * @return the array of indices of on-pixels in filledImage
         */
        inline cv::Mat PixelList() const
        {
            return pixelList;
        }
        /**
         * @brief setPixelList set pixelList to _pixelList
         * @param _pixelList
         */
        inline void setPixelList(const cv::Mat &_pixelList)
        {
            pixelList = _pixelList;
        }
        /**
         * @brief ConvexImage
         * @return the image where convex hull region is white and others are black
         */
        inline cv::Mat ConvexImage() const
        {
            return convexImage;
        }
        /**
         * @brief setConvexImage set convexImage to _convexImage
         * @param _convexImage
         */
        inline void setConvexImage(const cv::Mat &_convexImage)
        {
            convexImage = _convexImage;
        }
        /**
         * @brief MaxVal
         * @return the max intensity in the contour region
         */
        inline double MaxVal() const
        {
            return maxval;
        }
        /**
         * @brief setMaxVal set maxval to _maxval
         * @param _maxval
         */
        inline void setMaxVal(const double &_maxval)
        {
            maxval = _maxval;
        }
        /**
         * @brief MinVal
         * @return the min intensity in the contour region
         */
        inline double MinVal() const
        {
            return minval;
        }
        /**
         * @brief setMinVal set minval to _minval
         * @param _minval
         */
        inline void setMinVal(const double &_minval)
        {
            minval = _minval;
        }
        /**
         * @brief MaxLoc
         * @return the max.intensity pixel location
         */
        inline cv::Point MaxLoc() const
        {
            return maxloc;
        }
        /**
         * @brief setMaxLoc set maxloc to _maxLoc
         * @param _maxloc
         */
        inline void setMaxLoc(const cv::Point &_maxloc)
        {
            maxloc = _maxloc;
        }
        /**
         * @brief MinLoc
         * @return the min.intensity pixel location
         */
        inline cv::Point MinLoc() const
        {
            return minloc;
        }
        /**
         * @brief setMinLoc set minloc to _minloc
         * @param _minloc
         */
        inline void setMinLoc(const cv::Point &_minloc)
        {
            minloc = _minloc;
        }
        /**
         * @brief MeanVal
         * @return the mean intensity in the contour region
         */
        inline cv::Scalar MeanVal() const
        {
            return meanval;
        }
        /**
         * @brief setMeanVal set meanval to _meanval
         * @param _meanval
         */
        inline void setMeanVal(const cv::Scalar &_meanval)
        {
            meanval = _meanval;
        }
        /**
         * @brief Extreme
         * @return the extremal points in the region, respectively: rightMost, leftMost, topMost, bottomMost
         */
        inline std::vector<cv::Point> Extrema() const
        {
            return extrema;
        }
        /**
         * @brief setExtrema set extrame to _extrema
         * @param _extrema
         */
        inline void setExtrema(const std::vector<cv::Point> &_extrema)
        {
            extrema = _extrema;
        }
        /**
         * @brief Solidity
         * @return solidity = contour area / convex hull area
         */
        inline double Solidity() const
        {
            return solidity;
        }
        /**
         * @brief setSolidity set solidity to _solidity
         * @param _solidity
         */
        inline void setSolidity(const double &_solidity)
        {
            solidity = _solidity;
        }

    private:
        double area, perimeter;
        cv::Moments moments;
        cv::Point centroid;
        cv::Rect boundingBox;
        double aspect_ratio, equi_diameter, extent;
        std::vector< cv::Point> convex_hull;
        double convex_area, solidity;
        cv::Point center;
        double majoraxis_length, minoraxis_length;
        double orientation, eccentricity;
        cv::Mat filledImage, pixelList;
        double filledArea;
        cv::Mat convexImage;
        cv::RotatedRect ellipse;
        std::vector<cv::Point> approx;
        double maxval, minval;
        cv::Point maxloc, minloc;
        cv::Scalar meanval;
        std::vector<cv::Point> extrema;
};

#endif
