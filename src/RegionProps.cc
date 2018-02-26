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

#include "regionprops.h"
#include <iostream>


RegionProps::RegionProps(const std::vector<cv::Point> &_contour,
                         const cv::Mat &_img)
    : img(_img.clone()), contour(_contour)
{
    compute();
}

void RegionProps::compute()
{
    region.setArea(area());
    region.setPerimeter(perimeter());
    region.setMoments(moments());
    region.setCentroid(centroid());
    region.setAspectRatio(aspectratio());
    region.setBoundingBox(boundingbox());
    region.setConvexHull(convex_hull());
    region.setConvexArea(convexarea());
    region.setEllipse(ellipse());
    region.setSolidity(solidity());
    region.setMajorAxis(majoraxislength());
    region.setMinorAxis(minoraxislength());
    region.setOrientation(orientation());
    region.setEccentricity(eccentricity());
    region.setApprox(approx());
    region.setFilledImage(filledimage());
    region.setFilledArea(filledarea());
    region.setPixelList(pixellist());
    region.setConvexImage(conveximage());
    pixelparameters();
    region.setExtrema(extrema());
}

double RegionProps::area()
{
    return cv::contourArea(contour);
}

double RegionProps::perimeter()
{
    return cv::arcLength(contour, true);
}

cv::Moments RegionProps::moments()
{
    return cv::moments(contour);
}

cv::Point RegionProps::centroid()
{
    cv::Point c(0, 0);
    if(region.Moments().m00 != 0.0)
    {
       c.x = region.Moments().m10 / region.Moments().m00;
       c.y = region.Moments().m01 / region.Moments().m00;
    }
    return c;
}

double RegionProps::aspectratio()
{
    return region.BoundingBox().width / double(region.BoundingBox().height);
}

double RegionProps::equivalentdiameter()
{
    return std::sqrt(4*region.Area()/CV_PI);
}

cv::Rect RegionProps::boundingbox()
{
    return cv::boundingRect(contour);
}

double RegionProps::extent()
{
    return region.Area()/(region.BoundingBox().width*region.BoundingBox().height);
}

std::vector<cv::Point> RegionProps::convex_hull()
{
    std::vector<cv::Point> convex;
    cv::convexHull(contour, convex);
    return convex;
}

double RegionProps::convexarea()
{
    return cv::contourArea(region.ConvexHull());
}

double RegionProps::solidity()
{
    return region.Area() / double(region.ConvexArea());
}

cv::RotatedRect RegionProps::ellipse()
{
    return cv::fitEllipse(contour);
}

double RegionProps::majoraxislength()
{
    return cv::max(region.Ellipse().size.width, region.Ellipse().size.height);
}

double RegionProps::minoraxislength()
{
    return cv::min(region.Ellipse().size.width, region.Ellipse().size.height);
}

double RegionProps::orientation()
{
    return region.Ellipse().angle;
}

double RegionProps::eccentricity()
{
    return std::sqrt(1 - (region.MinorAxis() / region.MajorAxis()) *
                     (region.MinorAxis() / region.MajorAxis()));
}

std::vector<cv::Point> RegionProps::approx()
{
    std::vector<cv::Point> a;
    cv::approxPolyDP(contour, a, 0.02*region.Perimeter(), true);
    return a;
}

cv::Mat RegionProps::filledimage()
{
    cv::Mat filled = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::drawContours(filled, std::vector< std::vector<cv::Point> >(1, contour), -1, cv::Scalar(255), -1);
    return filled;
}

double RegionProps::filledarea()
{
    return cv::countNonZero(region.FilledImage());
}

cv::Mat RegionProps::pixellist()
{
    cv::Mat locations;
    cv::findNonZero(region.FilledImage(), locations);
    return locations.t();
}

cv::Mat RegionProps::conveximage()
{
    cv::Mat convex = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::drawContours(convex, std::vector< std::vector<cv::Point> >(1,region.ConvexHull()), -1, 255, -1);
    return convex;
}

void RegionProps::pixelparameters()
{
    double minval, maxval;
    cv::Point minloc, maxloc;
    cv::Scalar meanval;
    cv::minMaxLoc(img, &minval, &maxval, &minloc, &maxloc, region.FilledImage());
    meanval = cv::mean(img, region.FilledImage());
    region.setMinLoc(minloc);
    region.setMaxLoc(maxloc);
    region.setMinVal(minval);
    region.setMaxVal(maxval);
    region.setMeanVal(meanval);
}

std::vector<cv::Point> RegionProps::extrema()
{
    std::vector<cv::Point>::iterator it;
    std::vector<cv::Point> e;
    cv::Point leftMost(img.cols, 0), rightMost(0,0),
            topMost(0, 0), bottomMost(0, img.rows);
    for(it = contour.begin(); it != contour.end(); ++it)
    {
        if((*it).x > rightMost.x)
        {
            rightMost.x = (*it).x;
            rightMost.y = (*it).y;
        }

        if((*it).x  < leftMost.x)
        {
            leftMost.x = (*it).x;
            leftMost.y = (*it).y;
        }

        if((*it).y > topMost.y)
        {
            topMost.x = (*it).x;
            topMost.y = (*it).y;
        }

        if((*it).y < bottomMost.y)
        {
            bottomMost.x = (*it).x;
            bottomMost.y = (*it).y;
        }
    }
    e.push_back(rightMost);
    e.push_back(leftMost);
    e.push_back(topMost);
    e.push_back(bottomMost);
    return e;
}
