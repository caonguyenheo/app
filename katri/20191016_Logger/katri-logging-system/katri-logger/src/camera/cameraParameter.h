#ifndef CAMERAPARAMETER_H
#define CAMERAPARAMETER_H

#include "QFileInfo"
#include "QString"
#include "../appConfig.h"

class CameraParameter
{
public:
    CameraParameter();
    CameraParameter(const QString &id,
                    const QString &src,
                    const QString &size,
                    const QString &format,
                    const QString &fileName,
                    const QString &path);
    ~CameraParameter();
    CameraParameter & operator=(const CameraParameter & rhs);

    QString getSrc() const;
    void setSrc(const QString src);

    QString getSize() const;
    void setSize(const QString size);

    QString getFormat() const;
    void setFormat(const QString format);

    QString getPath() const;
    void setPath(const QString path);

    QString getProgram() const;

    void setId(const QString id);
    QString getId() const;

    void setFileName(const QString fileName);
    QString getFileName() const;

    QString getDataSetPath() const;
    QString getFilePath() const;
    QString getTimestampPath() const;

private:
    QString _src;
    QString _size;
    QString _format;
    QString _path;
    const QString _program = "./yavta";
    QString _id;
    QString _fileName;
    int _datasetIndex;
};

#endif // CAMERAPARAMETER_H
