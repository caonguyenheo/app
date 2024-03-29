/*
*
* This file is part of QMapControl,
* an open-source cross-platform map widget
*
* Copyright (C) 2007 - 2008 Kai Winter
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with QMapControl. If not, see <http://www.gnu.org/licenses/>.
*
* Contact e-mail: kaiwinter@gmx.de
* Program URL   : http://qmapcontrol.sourceforge.net/
*
*/

#pragma once

// Local includes.
#include "MapViewer/osm/qmapcontrol_global.h"
#include "MapViewer/osm/Projection.h"

/*!
 * @author Chris Stylianou <chris5287@gmail.com>
 */
namespace qmapcontrol
{
    class  ProjectionEquirectangular : public Projection
    {
    public:
        //! Constuctor.
        /*!
         * Projection Equirectangular (EPSG:4326 - lat/long) constructor.
         */
        ProjectionEquirectangular() { } /// = default; @todo re-add once MSVC supports default/delete syntax.

        //! Disable copy constructor.
        ///ProjectionEquirectangular(const ProjectionEquirectangular&) = delete; @todo re-add once MSVC supports default/delete syntax.

        //! Disable copy assignment.
        ///ProjectionEquirectangular& operator=(const ProjectionEquirectangular&) = delete; @todo re-add once MSVC supports default/delete syntax.

        //! Destructor.
        ~ProjectionEquirectangular() { } /// = default; @todo re-add once MSVC supports default/delete syntax.

        /*!
         * The number of tiles on the x-axis for a given zoom.
         * @param zoom The zoom level.
         * @return number of tiles on the x-axis for a given zoom.
         */
        int tilesX(const int& zoom) const final;

        /*!
         * The number of tiles on the y-axis for a given zoom.
         * @param zoom The zoom level.
         * @return number of tiles on the y-axis for a given zoom.
         */
        int tilesY(const int& zoom) const final;

        /*!
         * Fetch the recognised EPSG number for the projection.
         * @return the recognised EPSG number for the projection.
         */
        int epsg() const final;

        /*!
         * Converts a world coorindate point (longitude/latitude) into the pixel point for a given zoom.
         * @param point_coord The world coordinate point to convert (longitude/latitude).
         * @param zoom The zoom level.
         * @return the converted world pixel point.
         */
        PointWorldPx toPointWorldPx(const PointWorldCoord& point_coord, const int& zoom) const final;

        /*!
         * Converts a world pixel point into the coorindate point (longitude/latitude) for a given zoom.
         * @param point_px The world pixel point to convert.
         * @param zoom The zoom level.
         * @return the converted world coorindate point (longitude/latitude).
         */
        PointWorldCoord toPointWorldCoord(const PointWorldPx& point_px, const int& zoom) const final;

    private:
        //! Disable copy constructor.
        ProjectionEquirectangular(const ProjectionEquirectangular&); /// @todo remove once MSVC supports default/delete syntax.

        //! Disable copy assignment.
        ProjectionEquirectangular& operator=(const ProjectionEquirectangular&); /// @todo remove once MSVC supports default/delete syntax.
    };
}
