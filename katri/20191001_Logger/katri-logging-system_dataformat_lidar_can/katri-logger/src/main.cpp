#include "QCoreApplication"
#include "QString"
#include "controller.h"
#include "signal.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"

/* Signal Handler for SIGINT */
[[ noreturn ]]void sigintHandler(int sig_num)
{
    Q_UNUSED(sig_num);
    NodeWorker::shutDown();
    fflush(stdout);
    exit(0);
}

int main(int argc, char *argv[])
{
    signal(SIGINT, sigintHandler);

    QCoreApplication a(argc, argv);

    // call once
    LoggerManager::init();

    // Force flush of the stdout buffer.
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    Controller controller(argc, argv);
    controller.startWork();

    return a.exec();
}
