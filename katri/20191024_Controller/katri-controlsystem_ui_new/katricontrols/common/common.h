#ifndef COMMON_H
#define COMMON_H

#include "QColor"
#include "katricontrols_export.h"

#define ENABLE_UI_OLD       (0)

enum class KATRICONTROLS_EXPORT RelayStatus : uint8_t {
    NONE      = 0x00,
    RELAYOK   = 0x01,
    RELAYFAIL = 0x02
};

enum Deviced {
    GPS = 0x01,
    LIDAR = 0x03,
    CANBUS = 0x07,
    CAMERA = 0x08
};

const QColor PLAYERBORDERCOLOR     = QColor(114,114,114);
const QColor PLAYERBACKGROUNDCOLOR = QColor(39,39,39);
const QColor MAINPANELBORDERCOLOR  = QColor(100,100,100);
const QColor MAINPANELBORDERCOLORCONTROLUI  = QColor(255,255,255);

QString convertColorToString(const QColor &color);
const QStringList NAME_CANBUS = QStringList() << "C-CAN" << "MRR" << "Mobileye" << "SRR-1F" << "SRR-2F" << "SRR-1R" << "SRR-2R";
const QStringList NAME_CAMERA = QStringList() << "Cam 1" << "Cam 2" << "Cam 3" << "Cam 4" << "Cam 5" << "Cam 6" << "Cam 7" << "Cam 8";
const QStringList NAME_LIDAR =  QStringList()<<"Lidar 1" << "Lidar 2" << "Lidar 3";
const QStringList NAME_GPS =  QStringList()<<"";
const QString NAME_TABRECORD = "Record";
const QString NAME_TABSYSTEMSTATUS = "System Status";
const QString NAME_TABRELAYALL = "Relay";

enum CheckboxType {
    FRONT_LIDAR = 0,
    TOP_LIDAR,
    REAR_LIDAR,
    CAN_SERIAL,
    MOBILEYE_SERIAL,
    WEATHE_SERIAL,
    FRONTCENTER_RADAR,
    FRONTLEFT_RADAR,
    FRONTRIGHT_RADAR,
    REARLEFT_RADAR,
    REARRIGHT_RADAR,
    FRONT_FHD,
    LEFT_FHD,
    RIGHT_FHD,
    REAR_FHD,
    FRONT_SVM,
    LEFT_SVM,
    RIGHT_SVM,
    REAR_SVM
};

#endif // COMMON_H
