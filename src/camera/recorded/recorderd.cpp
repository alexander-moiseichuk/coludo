/*
 * coludo project, uart flight recorder
 *
 * copyright (c) 2026, Leonid Moiseichuk under MIT license
 *
 * the idea of this part to replace https://www.dfrobot.com/product-2499.html which records on vfat sd card
 * with Luckfox Pico Mini video recorder which already used.
 *
 * the protocol is simple: <TAG>filename<TAG>contents to be added\n received from /dev/ttySx will be added to
 * file <RECORDER_FOLDER>/filename
 *
 * no optimizations and files flushed and closed every time as part of cost for stability.
 *
 * lane itself must be tweaked on both sides as e.g.
 *  stty -F /dev/ttySx 115200 cs8 -parenb -cstopb -ixon
 * or for short lanes and optimistic case
 *  stty -F /dev/ttySx 921600 cs8 -parenb -cstopb -ixon
 *
 * launch parameters:
 * - serial line device
 * - folder to store recorded files
 * - default name of recorded file
 * - tag for recorded file name switch in the beginning of file, optional
 *
 * example:
 *  recorded /dev/ttySx /userdata/recordings recorder.log @
 */

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

/*
 * pre-conditions and parameters to use
 */

static const char EOL = '\n';   // end of line marker
static const char EOS = '\0';   // end of string for C

static const unsigned DEVICE_BUFFER_SIZE = 256 * 1024;  // more than enough for any speed including 921600 bod
static char deviceBuffer[DEVICE_BUFFER_SIZE];           // no malloc - safer, no initialization as .bss

static const char* devicePath = nullptr;
static const char* recordingFolder = nullptr;
static const char* recordingFile = nullptr;
static const char* fileNameTag = nullptr;

// descriptors for device and default file
static int deviceFd = -1;
static FILE* recordingStream = nullptr;


/*
 * helpers and the rest
 */

#define VALID_STRING(s)     ((s) && *(s))


static void info(const char* format, ...) {
    if (format && *format) {
        va_list args;
        va_start(args, format);
        vsyslog(LOG_INFO, format, args);
        va_end(args);
    }
}

static FILE* openStream(const char* name) {
    char path[1024];
    snprintf(path, sizeof(path), "%s/%s", recordingFolder, name);
    FILE* stream = fopen(path, "at");
    if (!stream) {
        info("failed to open recording file %s [%d]: %s", path, errno, strerror(errno));
    }
    return stream;
}

static bool setup(const int argc, const char* argv[]) {
    if (argc < 4) {
        info("expected args: uart_device recording_folder default_file [TAG]");
        info("example: %s /dev/ttySx /userdata/recordings recorder.log @");
        return false;
    }
    if (VALID_STRING(argv[1])) {
        devicePath = argv[1];
    } else {
        info("cannot recognize uart device path %s", argv[1]);
        return false;
    }
    if (VALID_STRING(argv[2])) {
        recordingFolder = argv[2];
    } else {
        info("cannot recognize recording folder path %s", argv[2]);
        return false;
    }
    if (VALID_STRING(argv[3])) {
        recordingFile = argv[3];
    } else {
        info("cannot recognize default recording file path %s", argv[3]);
        return false;
    }
    if (argc >= 4 && VALID_STRING(argv[4])) {
        fileNameTag = argv[4];
    } else {
        info("the file tag was not set - will write everything to default file path %s/%s", recordingFolder, recordingFile);
    }
    // check and open device
    deviceFd = open(devicePath, O_RDONLY|O_CLOEXEC|O_NOATIME);
    if (deviceFd < 0) {
        info("failed to open device file %s [%d]: %s", devicePath, errno, strerror(errno));
        return false;
    }
    // and now for default file for recording
    return (recordingStream = openStream(recordingFile)) != nullptr;
}

static void shutdown() {
    static const char* mark[] = {
        "FAIL", // != 0
        "PASS", // == 0
    };
    // just in case but system will do it for us
    if (recordingStream) {
        info("default recording stream closed: %s", mark[0 == fclose(recordingStream)]);
        recordingStream = nullptr;
    }
    if (deviceFd >= 0) {
        info("uart device stream closed: %s", mark[0 == close(deviceFd)]);
        deviceFd = -1;
    }
}

static bool appendContents(const char* name, const char* contents) {
    bool pass = false;
    FILE* stream = openStream(name);
    if (stream) {
        pass = (fputs(contents, stream) > 0);
        // close must be called always if open succeeed
        if (fclose(stream)) {
            pass = false;   // if fclose has problems it must be indicated dropping previous success
        }
    }
    return pass;
}

static void processBuffer(char* buffer) {
    const char* fileName = nullptr;
    // flush buffer with EOL and EOS to default file or specified file
    if (fileNameTag) {
        // OK, we migh have file name first
        if (buffer[0] == fileNameTag[0]) {
            for (unsigned pos = 1; buffer[pos]; pos++) {
                if (fileNameTag[0] == buffer[pos] && pos > 3) {
                    // discovered fille close with some meaninful length
                    fileName = &buffer[1];
                    buffer[pos] = EOS;
                    buffer += pos + 1;
                    break;
                }
            } // for chars in buffer
        }
    }
    // here buffer position may change, check for at least non-empty
    if (VALID_STRING(buffer)) {
        if (fileName) {
            if (!appendContents(fileName, buffer)) {
                info("failed to add contents into %s/%s", recordingFolder, fileName);
            }
        } else {
            if (fputs(buffer, recordingStream) <= 0) {
                info("failed to add contents into %s/%s", recordingFolder, recordingFile);
            }
        }
    }
}


static void yield() {
    usleep(1);  // kind of CONFIG_HZ 1000 and max 1 line miss
}

static void processing() {
    size_t position = 0;
    size_t lineCounter = 0;
    char buf[1024];  // usually 64 bytes but lets make with spare

    info("reading data from %s %d", devicePath, deviceFd);
    while (true) {
        const ssize_t len = read(deviceFd, buf, sizeof(buf));
        if (len <= 0) {
            // OK, nothing to read yet
            yield();
            continue;
        }
        // something in buf processing line by line, accumulating in deviceBuffer
        for (size_t pos = 0; pos < len && position < sizeof(deviceBuffer) - 1; pos++) {
            const char cursor = buf[pos];
            // there are options:
            // - non-printed character below EOS - skip
            // - EOS => add & flush contents
            // - printed character - just append
            if (cursor < EOL) {
                continue;
            }
            if (EOL == cursor) {
                deviceBuffer[position++] = EOL;
                deviceBuffer[position++] = EOS;
                processBuffer(deviceBuffer);
                position = 0;
                lineCounter++;
                if (0 == (lineCounter % 1000)) {
                    fflush(recordingStream);
                    info("there are %lu lines saved from %s", lineCounter, devicePath);
                }
                continue;
            }
            // the rest just add
            deviceBuffer[position++] = cursor;
        }
    }
}

int main(const int argc, const char* argv[]) {

    openlog("recorderd", LOG_PID|LOG_PERROR, LOG_DAEMON);
    if (setup(argc, argv)) {
        processing();
        shutdown();
    }
    closelog();

    return 0;
}
