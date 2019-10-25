/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the Qt Data Visualization module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:GPL$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3 or (at your option) any later version
** approved by the KDE Free Qt Foundation. The licenses are as published by
** the Free Software Foundation and appearing in the file LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "scatterdatamodifier.h"
#include <QtDataVisualization/qscatterdataproxy.h>
#include <QtDataVisualization/qvalue3daxis.h>
#include <QtDataVisualization/q3dscene.h>
#include <QtDataVisualization/q3dcamera.h>
#include <QtDataVisualization/qscatter3dseries.h>
#include <QtDataVisualization/q3dtheme.h>
#include <QtCore/qmath.h>
#include <QtCore/qrandom.h>
#include <QtWidgets/QComboBox>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <QDir>
using namespace QtDataVisualization;

//#define RANDOM_SCATTER // Uncomment this to switch to random scatter

ScatterDataModifier::ScatterDataModifier(Q3DScatter *scatter)
    : m_graph(scatter),
      m_fontSize(1.0f),
      m_style(QAbstract3DSeries::MeshPoint)
      //m_smooth(true),
     // m_itemCount(lowerNumberOfItems),
     // m_curveDivider(lowerCurveDivider)
{
    m_graph->activeTheme()->setType(Q3DTheme::ThemeEbony);
    QFont font = m_graph->activeTheme()->font();
    font.setPointSize(m_fontSize);
    m_graph->activeTheme()->setFont(font);
    //m_graph->setShadowQuality(QAbstract3DGraph::ShadowQualitySoftLow);
    m_graph->scene()->activeCamera()->setCameraPreset(Q3DCamera::CameraPresetFront);

    QScatterDataProxy *proxy = new QScatterDataProxy;
    QScatter3DSeries *series = new QScatter3DSeries(proxy);
    series->setItemLabelFormat(QStringLiteral("@xTitle: @xLabel @yTitle: @yLabel @zTitle: @zLabel"));
    //series->setMeshSmooth(m_smooth);
    series->setItemSize(0.01f);
    m_graph->addSeries(series);
   // addData();

}

ScatterDataModifier::~ScatterDataModifier()
{
//    if(m_graph != nullptr)
//    {
//        delete m_graph;
//    }
}

void ScatterDataModifier::SetData(QVector<QVector3D> *dataArray, bool clearData)
{

    //m_graph->seriesList().at(0)->dataProxy()->resetArray(0);
    if(!clearData)
    {
      addData(dataArray);
    }
    else
    {
        Sleep(2);
        m_graph->seriesList().at(0)->dataProxy()->resetArray(0);
    }
}

void ScatterDataModifier::addData(QVector<QVector3D> *itemList)
{
    // Configure the axes according to the data
    //! [4]
    m_graph->axisX()->setTitle("X");
    m_graph->axisY()->setTitle("Y");
    m_graph->axisZ()->setTitle("Z");
    //! [4]

    QScatterDataArray *dataArray = new QScatterDataArray;
       dataArray->resize(itemList->count());
       QScatterDataItem *ptrToDataArray = &dataArray->first();
       for (int i = 0; i < itemList->count(); i++) {
           ptrToDataArray->setPosition(itemList->at(i));
           ptrToDataArray++;
       }
    Sleep(2);
    m_graph->seriesList().at(0)->dataProxy()->resetArray(dataArray);



//    dataArray->resize(size);

//    dataArray->append(&ptrToDataArray);

//     for (int i = 0; i < t_PCD.vListPoint.size(); i++)
//     {

//        point = t_PCD.vListPoint.at(i);
//         ptrToDataArray->setPosition(QVector3D(point.split(" ")[0].toFloat(), point.split(" ")[1].toFloat(), point.split(" ")[2].toFloat()));
//         ptrToDataArray++;

//         point.clear();
//     }



//#ifdef RANDOM_SCATTER
//    for (int i = 0; i < m_itemCount; i++) {
//        ptrToDataArray->setPosition(randVector());
//        ptrToDataArray++;
//    }
//#else
//    //! [6]
//    float limit = qSqrt(m_itemCount) / 2.0f;
//    for (float i = -limit; i < limit; i++) {
//        for (float j = -limit; j < limit; j++) {
//            ptrToDataArray->setPosition(QVector3D(i + 0.5f,
//                                                  qCos(qDegreesToRadians((i * j) / m_curveDivider)),
//                                                  j + 0.5f));
//            ptrToDataArray++;
//        }
//    }
//    //! [6]
//#endif



}

////! [8]
//void ScatterDataModifier::changeStyle(int style)
//{
//    QComboBox *comboBox = qobject_cast<QComboBox *>(sender());
//    if (comboBox) {
//        m_style = QAbstract3DSeries::Mesh(comboBox->itemData(style).toInt());
//        if (m_graph->seriesList().size())
//            m_graph->seriesList().at(0)->setMesh(m_style);
//    }
//}

//void ScatterDataModifier::setSmoothDots(int smooth)
//{
//    m_smooth = bool(smooth);
//    QScatter3DSeries *series = m_graph->seriesList().at(0);
//    series->setMeshSmooth(m_smooth);
//}

//void ScatterDataModifier::changeTheme(int theme)
//{
//    Q3DTheme *currentTheme = m_graph->activeTheme();
//    currentTheme->setType(Q3DTheme::Theme(theme));
//    emit backgroundEnabledChanged(currentTheme->isBackgroundEnabled());
//    emit gridEnabledChanged(currentTheme->isGridEnabled());
//    emit fontChanged(currentTheme->font());
//}

//void ScatterDataModifier::changePresetCamera()
//{
//    static int preset = Q3DCamera::CameraPresetFrontLow;

//    m_graph->scene()->activeCamera()->setCameraPreset((Q3DCamera::CameraPreset)preset);

//    if (++preset > Q3DCamera::CameraPresetDirectlyBelow)
//        preset = Q3DCamera::CameraPresetFrontLow;
//}

//void ScatterDataModifier::changeLabelStyle()
//{
//    m_graph->activeTheme()->setLabelBackgroundEnabled(!m_graph->activeTheme()->isLabelBackgroundEnabled());
//}

//void ScatterDataModifier::changeFont(const QFont &font)
//{
//    QFont newFont = font;
//    newFont.setPointSizeF(m_fontSize);
//    m_graph->activeTheme()->setFont(newFont);
//}

//void ScatterDataModifier::shadowQualityUpdatedByVisual(QAbstract3DGraph::ShadowQuality sq)
//{
//    int quality = int(sq);
//    emit shadowQualityChanged(quality); // connected to a checkbox in main.cpp
//}

//void ScatterDataModifier::changeShadowQuality(int quality)
//{
//    QAbstract3DGraph::ShadowQuality sq = QAbstract3DGraph::ShadowQuality(quality);
//    m_graph->setShadowQuality(sq);
//}

//void ScatterDataModifier::setBackgroundEnabled(int enabled)
//{
//    m_graph->activeTheme()->setBackgroundEnabled((bool)enabled);
//}

//void ScatterDataModifier::setGridEnabled(int enabled)
//{
//    m_graph->activeTheme()->setGridEnabled((bool)enabled);
//}
////! [8]

//void ScatterDataModifier::toggleItemCount()
//{
//    if (m_itemCount == numberOfItems) {
//        m_itemCount = lowerNumberOfItems;
//        m_curveDivider = lowerCurveDivider;
//    } else {
//        m_itemCount = numberOfItems;
//        m_curveDivider = curveDivider;
//    }
//    m_graph->seriesList().at(0)->dataProxy()->resetArray(0);
//    //addData();
//}

//QVector3D ScatterDataModifier::randVector()
//{
//    return QVector3D(
//                (float)(QRandomGenerator::global()->bounded(100)) / 2.0f -
//                (float)(QRandomGenerator::global()->bounded(100)) / 2.0f,
//                (float)(QRandomGenerator::global()->bounded(100)) / 100.0f -
//                (float)(QRandomGenerator::global()->bounded(100)) / 100.0f,
//                (float)(QRandomGenerator::global()->bounded(100)) / 2.0f -
//                (float)(QRandomGenerator::global()->bounded(100)) / 2.0f);
//}
