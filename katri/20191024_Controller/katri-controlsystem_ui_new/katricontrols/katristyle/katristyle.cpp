#include "katristyle.h"
#include <QFile>
#include <QTextStream>

QString KatriStyle::getExportFilecss()
{
    QFile styleFile(":/style/style/styleSheet.css");
    QString styleSheet;
    if(styleFile.open(QIODevice::ReadOnly))
    {
        QTextStream textStream(&styleFile);
        styleSheet = textStream.readAll();
        styleFile.close();
    }
    return styleSheet;
}
