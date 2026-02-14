#ifndef LOG_H
#define LOG_H

#include <iostream>
#include <string>

enum class LogLevel{INFO, WARNING, ERROR}; // error level will exit

inline void log(LogLevel level, const std::string& message){
    switch(level){
        case LogLevel::INFO:
            std::cout << "[INFO] " << message << std::endl;
            break;
        case LogLevel::WARNING:
            std::cout << "\033[33m[WARNING]\033[0m " << message << std::endl;
            break;
        case LogLevel::ERROR:
            std::cerr << "\033[31m[ERROR]\033[0m " << message << std::endl;
            std::exit(EXIT_FAILURE);
    }
}

#endif