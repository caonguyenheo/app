#ifndef KITTIEXTRACTOR_H
#define KITTIEXTRACTOR_H

#include <QObject>
#include <QDateTime>

class KittiExtractor : public QObject
{
    Q_OBJECT

public:
    KittiExtractor(QObject *parent = nullptr,
                   QString basedKittiPath = "",
                   QString extKittiPath = "");
    ~KittiExtractor();

    bool getExtractingStatus() {return _isExtracting;}
    bool setBasedKittiPath(QString basedKittiPath);
    bool setExtractionKittiPath(QString extKittiPath);
    bool execExtracting(QString start, QString end);
    bool execExtracting(QDateTime start, QDateTime end);
    bool multiDataExtractor(QString orgDSPath, QDateTime start, QDateTime end);
    bool singleDataExtractor(QString orgDSPath, QDateTime start, QDateTime end);

private:
    bool    _isExtracting;
    QString _basedKittiPath;
    QString _extractionKittiPath;
    QDateTime _startTimestamp;
    QDateTime _endTimestamp;

};

#endif // KITTIEXTRACTOR_H
