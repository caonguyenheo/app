
#include <QDebug>
#include <archive.h>
#include <archive_entry.h>
#include "qarchive.h"

struct QArchive_internal
{
	int response;
	struct archive_entry * entry;
	struct archive * archive;

	QArchive_internal();
	~QArchive_internal();
};

QArchive_internal::QArchive_internal()
{
	archive = archive_read_new();
	archive_read_support_filter_all(archive);
	archive_read_support_format_all(archive);
}

QArchive_internal::~QArchive_internal()
{
	response = archive_read_free(archive);
}

QArchive::QArchive()
{
}

void QArchive::setSource(QString archive_file)
{
    if (data) {
        delete data;
    }
    data = new QArchive_internal();
    openArchive(archive_file);
}

bool QArchive::openArchive(QString archive_file) {
	QByteArray qba = archive_file.toUtf8();
    data->response = archive_read_open_filename(data->archive, qba.data(), BUFFER_SIZE);
    if(data->response != ARCHIVE_OK) return false;
	return true;
}

bool QArchive::listFiles(QStringList & out) {

    while(archive_read_next_header(data->archive, &data->entry) == ARCHIVE_OK) {
        out.append(QString(archive_entry_pathname(data->entry)));
	}
	return true;
}

bool QArchive::readFile(QString target, QByteArray & out)
{
	bool read_ok = false;
	bool found_file = false;
    while(!found_file && archive_read_next_header(data->archive, &data->entry) == ARCHIVE_OK) {
        QString f(archive_entry_pathname(data->entry));
		if(f == target) {
			found_file = true;
            read_ok = QArchive::readData(out);
		}
	}
	return read_ok;
}

bool QArchive::readFirstFile(QByteArray & out)
{
	bool read_ok = false;
    while(archive_read_next_header(data->archive, &data->entry) == ARCHIVE_OK) {
        if(archive_entry_filetype(data->entry) == AE_IFREG) {
            read_ok = QArchive::readData(out);
			break;
		}
	}
	return read_ok;
}

bool QArchive::readData(QByteArray & out)
{
	const void * buff;
	size_t size;
    la_int64_t offset;
	bool read_ok = false;
	bool still_reading = true;
	while(still_reading) {
        data->response = archive_read_data_block(data->archive, &buff, &size, &offset);
        if(data->response == ARCHIVE_EOF) {
			read_ok = true;
			still_reading = false;
		}
        else if(data->response != ARCHIVE_OK) {
			// some error
			still_reading = false;
		}
		else { // keep reading
			const char * p = static_cast<const char*>(buff);
            out.append(p, int(size));
		}
	}
	return read_ok;
}

QArchive &QArchive::instance()
{
    static QArchive instance;
    return instance;
}
