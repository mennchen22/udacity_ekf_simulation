//
// Created by ulrich on 03.06.2021.
//


#include "logging.h"


void Logging::logging(const std::string &message, LOG_LEVEL level) {
    if (level < GlobalLogLevel) {
        return;
    }
    if (level == DEBUG) {
        std::cout << "[DEBUG] " << message << std::endl;
    } else if (level == INFO) {
        std::cout << "[INFO] " << message << std::endl;
    } else if (level == WARNING) {
        std::cout << "[WARNING] " << message << std::endl;
    } else if (level == ERROR) {
        std::cout << "[ERROR] " << message << std::endl;
    }
}


void Logging::SetLogLevel(LOG_LEVEL globalLevel) {
    GlobalLogLevel = globalLevel;
}

Logging::Logging() = default;
