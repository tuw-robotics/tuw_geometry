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


#ifndef TUW_GRID_MAP_H
#define TUW_GRID_MAP_H

#include <memory>
#include <tuw_geometry/world_scoped_maps.h>

namespace tuw
{

template <class T>
class GridMap : protected WorldScopedMaps
{
public:

    //special class member functions
    GridMap ( )
    {
        bool read_only_ = true;
    }
    virtual ~GridMap()                 = default;
    GridMap ( const GridMap& ) = default;
    GridMap& operator= ( const GridMap& ) = default;
    GridMap ( GridMap&& )      = default;
    GridMap& operator= ( GridMap&& )      = default;

    template <typename MapMetaData>
    void init ( const MapMetaData &metadata, T *data )
    {
        WorldScopedMaps::init ( metadata );
        data_ = cv::Mat_<T> ( height(), width(), data );
        read_only_ = false;
    }
    template <typename MapMetaData>
    void init ( const MapMetaData &metadata, const T *data )
    {
        WorldScopedMaps::init ( metadata );
        data_ = cv::Mat_<T> ( height(), width(), ( T * ) data );
        read_only_ = true;
    }

    const T& get ( const Point2D& _world_coordinates ) const
    {
        return data_.at( w2m ( _world_coordinates ) );
    }

private:
    bool read_only_;
    cv::Mat_<T> data_;
};
}
#endif // TUW_GRID_MAP_H
