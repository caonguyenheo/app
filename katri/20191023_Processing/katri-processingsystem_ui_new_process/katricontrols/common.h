#ifndef COMMON_H
#define COMMON_H

#include <QColor>
#include <QDateTime>

const QColor PLAYERBORDERCOLOR     = QColor(114,114,114);
const QColor PLAYERBACKGROUNDCOLOR = QColor(39,39,39);
const QColor MAINPANELBORDERCOLOR  = QColor(100,100,100);

// Constant definition
const QString TIMESTAMP_FORMAT = "yyyy-MM-dd HH:mm:ss.zzz";
const QString DATE_FORMAT      = "yyyy_MM_dd";

const int PROCESS_TIMEOUT = 28800000; // 8h * 3600s * 1000ms

// Data source folder prefix
const QStringList driveFilter= {"*_drive_*"};
//const QStringList gnssFilter("gnss_*");
const QStringList tsFilter("timestamps*");
const QStringList rawFilter("*.raw");

// Common functions
QString convertColorToString(const QColor &color);

void readFileBuffer(const char* path, QStringList &output);

void readMinMaxTimestamp(const char* path, QStringList &rangeList);

QMap<int,int> *makeCommonTimeline(QString first, QString last);

bool isInRange(int less, int more, int current);

enum EventType {
    NEW_TIMELINE,
    TIMELINE_VALUE,
    TIMELINE_SEEK,
    PLAY,    // Playing state
    PAUSE,   // Paused state
    EOP,     // End of play
    RENDER_STATE,
    BLACKSCREEN
};

enum ScreenType {
    stALL,
    stCAMERA,
    stSVM,
    stGPS,
    stRADAR,
    stLIDAR,
    stCANBUS
};


struct ImageData
{
    QVector<QString> vImageDataPaths;
    QVector<qint32> timestamps;
};

enum ImagePanel
{
    NON_IMAGE_PANEL = -1,
    FIRST_PANEL,
    SECOND_PANEL,
    THIRD_PANEL,
    FOURTH_PANEL
};

struct LidarData
{
    QVector<QString> vlidarData;
    QVector<qint32> timestamps;
};

enum LidarPanel
{
    NON_PANEL = -1,
    LEFT_PANEL,
    RIGHT_PANEL,
    BOTTOM_PANEL,
};

class EventObject
{
};

class TimelineObject : public EventObject
{
public:
    QString start;
    QString end;
    QString dataPath;
};

class TimelineValue : public EventObject
{
public:
    int value;
};

class StateEvent : public EventObject
{
public:
    QString source_id;
    bool status;
};

//EventObject
#endif // COMMON_H
