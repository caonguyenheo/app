#ifndef COMMON_GPS_H
#define COMMON_GPS_H

#include <QList>

const QStringList dataformatgps = QStringList()<<"lat:    latitude of the oxts-unit (deg)"
                                            <<"lon:    longitude of the oxts-unit (deg)"
                                            <<"alt:    altitude of the oxts-unit (m)"
                                            <<"roll:   roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi"
                                            <<"pitch:  pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2"
                                            <<"yaw:    heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi"
                                            <<"vn:     velocity towards north (m/s)"
                                            <<"ve:     velocity towards east (m/s)"
                                            <<"vf:     forward velocity, i.e. parallel to earth-surface (m/s)"
                                            <<"vl:     leftward velocity, i.e. parallel to earth-surface (m/s)"
                                            <<"vu:     upward velocity, i.e. perpendicular to earth-surface (m/s)"
                                            <<"ax:     latitude of the oxts-unit (deg)"
                                            <<"ay:     acceleration in y, i.e. in direction of vehicle left (m/s^2)"
                                            <<"ay:     acceleration in z, i.e. in direction of vehicle top (m/s^2)"
                                            <<"af:     forward acceleration (m/s^2)";

const QStringList dataformatlidar = QStringList()<<"OBJ: first developed by Wavefront technologies, the format has been adopted by a wide range of 3D graphics applications. These include Bentley Systems, RealityCapture and Trimble. It is a simple data format that only represents 3D geometry, normals, colour and texture. It is commonly ASCII, however there are some proprietary binary versions of OBJ"
                                                <<"PLY: known as the polygon file format or Stanford triangle format, PLY was inspired by OBJ and purpose-built to store 3D data. PLY  uses lists of nominally flat polygons to represent objects. The goal was to add extensibility capabilities and the ability to store a greater number of physical elements. The result is a file format capable of representing colour, transparency, surface normals, texture, coordinates and data confidence values. There are two versions of this file, one in ASCII and the other binary."
                                                <<"XYZ: is a non-standardised set of files based on Cartesian coordinates (‘x’ ‘y’ and ‘z’). XYZ is an archetypal ASCII file type, conveying data in lines of text. There are no unit standardisations for XYZ files. Although there is wide compatibility across programs for this type of file, the lack of standardisation surrounding units and specifications makes it a fundamentally faulty method of data transfer unless additional information is supplied."
                                                <<"PCG, RCS, RCP: are all file formats developed by Autodesk to specifically meet the demands of their software suite. RCS and RCP are newer. Autodesk products, however, are often able to convert some open formats, such as PTS into PCG files."
                                                <<"E57: is a vendor-neutral file format for point cloud storage. It can also be used to store images and metadata produced by laser scanners and other 3D imaging systems. It is compact and widely used. It also utilises binary code in tandem with ASCII, providing much of the accessibility of ASCII and the speed of binary. E57 can represent normals, colours and scalar field intensity.";

#endif // COMMON_GPS_H
