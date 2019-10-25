#include "checkedbox.h"
#include <QDebug>


checkedbox::checkedbox(QWidget *parent) : QWidget(parent)
{

}

void checkedbox::getcheckedbox(CheckboxType type)
{
    switch (type) {
    case CheckboxType::FRONT_LIDAR:
        qDebug()<<"FRONT_LIDAR";
        break;
    case CheckboxType::TOP_LIDAR:
        qDebug()<<"TOP_LIDAR";
        break;
    case CheckboxType::REAR_LIDAR:
        qDebug()<<"REAR_LIDAR";
        break;
    case CheckboxType::CAN_SERIAL:
        qDebug()<<"CAN_SERIAL";
        break;
    case CheckboxType::MOBILEYE_SERIAL:
        qDebug()<<"MOBILEYE_SERIAL";
        break;
    case CheckboxType::FRONTCENTER_RADAR:
        qDebug()<<"FRONTCENTER_RADAR";
        break;
    case CheckboxType::FRONTLEFT_RADAR:
        qDebug()<<"FRONTLEFT_RADAR";
        break;
    case CheckboxType::FRONTRIGHT_RADAR:
        qDebug()<<"FRONTRIGHT_RADAR";
        break;
    case CheckboxType::REARLEFT_RADAR:
        qDebug()<<":REARLEFT_RADAR";
        break;
    case CheckboxType::REARRIGHT_RADAR:
        qDebug()<<"REARRIGHT_RADAR";
        break;
    case CheckboxType::FRONT_FHD:
        qDebug()<<"FRONT_FHD";
        break;
    case CheckboxType::LEFT_FHD:
        qDebug()<<"FRONT_LIDAR";
        break;
    case CheckboxType::RIGHT_FHD:
        qDebug()<<"RIGHT_FHD";
        break;
    case CheckboxType::REAR_FHD:
        qDebug()<<"REAR_FHD";
        break;
    case CheckboxType::FRONT_SVM:
        qDebug()<<"FRONT_SVM";
        break;
    case CheckboxType::LEFT_SVM:
        qDebug()<<"LEFT_SVM";
        break;
    case CheckboxType::RIGHT_SVM:
        qDebug()<<"RIGHT_SVM";
        break;
    case CheckboxType::REAR_SVM:
        qDebug()<<"REAR_SVM";
        break;
    default:
        break;
    }
}
