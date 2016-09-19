/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by Horatiu George Todoran <todorangrg@gmail.com>   *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#include <tuw_geometry/layered_maps.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/opencv.hpp>
#include <opencv/cv.hpp>

using namespace tuw;
using namespace std;
using namespace cv;

LayeredMaps::LayeredMaps () {

}

void LayeredMaps::initLayers( int width_pixel, int height_pixel, 
		              double min_x, double max_x, 
		              double min_y, double max_y, double rotation  ) {    
    init(width_pixel, height_pixel, min_x, max_x, min_y, max_y, rotation);
    initLayers();
}

size_t LayeredMaps::sizeLayers() const {
    return mapLayers_.size();
}


void LayeredMaps::resizeLayers( const size_t& _n ) {
    mapLayers_.resize ( _n );
}

void LayeredMaps::initLayers() {
    for(size_t i = 0; i < mapLayers_.size(); i++){ mapLayers_[i].create ( width(), height(), CV_8UC1 ); clearLayer(i); }
}

void LayeredMaps::clearLayers() {
    for(size_t i = 0; i < mapLayers_.size(); i++){ clearLayer(i); }
}
void LayeredMaps::clearLayer( const size_t& _layer ) {
    mapLayers_[_layer].setTo( 0xFF );
}

cv::Mat& LayeredMaps::mapLayer( const size_t& _layer ) {
    return mapLayers_[_layer];
}
const cv::Mat& LayeredMaps::mapLayer( const size_t& _layer ) const {
    return mapLayers_[_layer];
}

double LayeredMaps::getVal( const size_t& _layer, const Point2D& _worldPos, Interpolation _interpolationType ) const {
    
    if ( !_worldPos.inside(min_x(), min_y(),  max_x(), max_y() ) ) { return -100; }
    
    Point2D mapPos = w2m(_worldPos);
    const double mapPos_x_d = mapPos.x()       , mapPos_y_d = mapPos.y();
    const int    mapPos_x_i = round(mapPos_x_d), mapPos_y_i = round(mapPos_y_d);
    
    double retVal;
    switch ( _interpolationType ) {
	case SIMPLE : 
	    retVal = mapLayer(_layer).at<uint8_t>( mapPos_y_i, mapPos_x_i ); 
	    break;
	case BILINEAR: 
	    //if((mapPos_y_i+1 >= mapLayer(layer).rows)||(mapPos_x_i+1 >= mapLayer(layer).cols)||(mapPos_y_i <0)||(mapPos_x_i <0)){throw 0;}
	    
	    const double f00 = mapLayer(_layer).at<uint8_t>(mapPos_y_i    , mapPos_x_i    );
	    const double f10 = mapLayer(_layer).at<uint8_t>(mapPos_y_i    , mapPos_x_i + 1);
	    const double f01 = mapLayer(_layer).at<uint8_t>(mapPos_y_i + 1, mapPos_x_i    );
	    const double f11 = mapLayer(_layer).at<uint8_t>(mapPos_y_i + 1, mapPos_x_i + 1);
	    const double mapPosRed_x = mapPos_x_d - mapPos_x_i, mapPosRed_y = mapPos_y_d - mapPos_y_i;
	    
	    retVal =  f00 * (1 - mapPosRed_x) * (1 - mapPosRed_y) + 
		      f10 *      mapPosRed_x  * (1 - mapPosRed_y) + 
		      f01 * (1 - mapPosRed_x) *      mapPosRed_y  + 
		      f11 *      mapPosRed_x  *      mapPosRed_y;
	    break;
    }
    return retVal;
}

float distance2probabilitySigmoid ( float d, float threshold ) {
    float p = 0;
    if ( d < threshold ) {
        p = tanh ( ( 2*M_PI*-d ) /threshold+M_PI ) * 0.5 + 0.5;
    }
    return p;
}
float distance2probabilityTriangle ( float d, float threshold ) {
    float p = 0;
    if ( d < threshold ) {
        p = 1.-d/threshold;
    }
    return p;
}

void LayeredMaps::computeDistanceField ( Mat& _mDst, vector< Point2D >& _pSrc, const double& _radius, bool _flipDistance, bool connectPoints ) const {
    Mat srcMap;  srcMap.create ( width(), height(), CV_8UC1 ); srcMap.setTo ( 0xFF );
    Scalar colour ( 0 );
    if(connectPoints){ for ( size_t i = 1; i < _pSrc.size(); i++ ) { line   ( srcMap, _pSrc[i-1], _pSrc[i], colour, 8); } } 
    else             { for ( size_t i = 0; i < _pSrc.size(); i++ ) { circle ( srcMap, _pSrc[i], 1, colour, 1, 8); } }
    computeDistanceField ( _mDst, srcMap, _radius, _flipDistance );
}


void LayeredMaps::computeDistanceField ( Mat& _mDst, Mat& _mSrc, const double& _radius, bool _flipDistance ) const {
    Mat srcMap; 
    if (_mSrc.channels() != 1)  { cvtColor(_mSrc, srcMap, CV_BGR2GRAY ); }
    else                        { srcMap = _mSrc;                        }
    /// Distance Transform
    int maskSize0 = CV_DIST_MASK_PRECISE;
    int voronoiType = -1;
    int distType0 = CV_DIST_L2;//CV_DIST_L1;
    int maskSize = voronoiType >= 0 ? CV_DIST_MASK_5 : maskSize0;
    int distType = voronoiType >= 0 ? CV_DIST_L2     : distType0;
    Mat destMap_f, destMap_ui,  labels;  
    destMap_f.create ( width(), height(), CV_32F );
    if ( voronoiType < 0 ){ cv::distanceTransform ( srcMap, destMap_f, distType, maskSize ); }
    else                  { cv::distanceTransform ( srcMap, destMap_f, labels, distType, maskSize, voronoiType ); }
    destMap_f.convertTo(destMap_ui, CV_8U );

    CV_Assert(destMap_ui.depth() == CV_8U);
    int channels = destMap_ui.channels();
    int nRows    = destMap_ui.rows;
    int nCols    = destMap_ui.cols * channels;
    if (destMap_ui.isContinuous()) { nCols *= nRows; nRows = 1; }

    float threshold = _radius * scale_x ();
    uint8_t* p_d; float* p_s;
    for(int i = 0; i < nRows; ++i) {
        p_d = destMap_ui.ptr<uint8_t>(i);
        for (int j = 0; j < nCols; ++j) { if (_flipDistance) { p_d[j] = (uint8_t)(255 *       distance2probabilityTriangle ( p_d[j], threshold )  ); }
	                                  else               { p_d[j] = (uint8_t)(255 * (1. - distance2probabilityTriangle ( p_d[j], threshold ) )); } }
    }
    if ( _mDst.channels() != 1 )  { cvtColor(destMap_ui, _mDst, CV_GRAY2BGR ); }
    else                          { _mDst = destMap_ui;                        }
    
}