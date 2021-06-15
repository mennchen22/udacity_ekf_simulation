//
// Created by ulrich on 03.06.2021.
//

#ifndef EXTENDEDKF_LOGGING_H
#define EXTENDEDKF_LOGGING_H
#include <string>
#include <iostream>

static int GlobalLogLevel = 0;
class Logging {
public:
    enum LOG_LEVEL {
        DEBUG = 0,
        INFO = 1,
        WARNING = 2,
        ERROR = 3
    };
    Logging();
    static void SetLogLevel(LOG_LEVEL globalLevel);
    static void logging(const std::string& message, LOG_LEVEL level);
};
#endif //EXTENDEDKF_LOGGING_H
