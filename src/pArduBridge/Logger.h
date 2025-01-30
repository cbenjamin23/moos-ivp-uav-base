#pragma once

#include <fstream>
#include <mutex>
#include <string>
#include <iomanip>
#include <chrono>

class Logger {
public:
    enum class Level {
        INFO,
        WARNING,
        ERROR
    };

    // Get singleton instance
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    // Configure log file (call once at startup)
    static void configure(const std::string& filename, bool append = true) {
        getInstance().openFile(filename, append);
    }

    // Log message with level
    void log(Level level, const std::string& message) {
        if(!enabled) return;
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_file.is_open()) {
            auto now = std::chrono::system_clock::now();
            auto time = std::chrono::system_clock::to_time_t(now);
            
            m_file << "[" << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S") << "] "
                   << levelToString(level) << ": " << message << std::endl;
        }
    }

    // Helper methods for different log levels
    static void info(const std::string& message) {
        getInstance().log(Level::INFO, message);
    }

    static void warning(const std::string& message) {
        getInstance().log(Level::WARNING, message);
    }

    static void error(const std::string& message) {
        getInstance().log(Level::ERROR, message);
    }

    static void enable(bool enb) {
        getInstance().enabled = enb;
    }

    bool enabled{false};
private:
    std::ofstream m_file;
    std::mutex m_mutex;

    
    
    Logger() = default;  // Private constructor
    ~Logger() {
        if (m_file.is_open()) {
            m_file.close();
        }
    }

    void openFile(const std::string& filename, bool append) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_file.is_open()) {
            m_file.close();
        }
        
        { // Clear file before appending
        std::ofstream ofs(filename, std::ios::out | std::ios::trunc);
        // Once ofs goes out of scope, the file is closed
        }

        m_file.open(filename, append ? std::ios::app : std::ios::out);
    }

    std::string levelToString(Level level) {
        switch(level) {
            case Level::INFO:    return "INFO";
            case Level::WARNING: return "WARNING";
            case Level::ERROR:   return "ERROR";
            default:             return "UNKNOWN";
        }
    }
};