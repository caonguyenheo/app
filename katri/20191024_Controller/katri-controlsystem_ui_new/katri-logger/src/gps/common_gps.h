#ifndef COMMON_GPS_H
#define COMMON_GPS_H

#include <QList>

const QStringList dataformat = QStringList()<<"lat:    latitude of the oxts-unit (deg)"
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
#endif // COMMON_GPS_H
