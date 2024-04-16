#ifndef STRUCT_H
#define STRUCT_H
#include <QString>
#include <deque>
#include <vector>
enum Cmd{
    stationList_ = 0,
    nextstationList_ = 1,
    relocation_   = 2,
    start_       = 3,
    pause_       = 4,
    resume_      = 5,
    stop_        = 6,
    param_       = 7
};

struct Station{
    int stationId;
    double lon;
    double lat;
    double azi;
    QString stationName;
};

struct StationList{
    bool isstart;
    QString method;
    std::deque<Station> stationlist;
    StationList& operator=(const StationList&);
};

struct NextStationList{
    QString method;
    std::deque<Station> stationlist;
     NextStationList& operator=(const NextStationList&);
};

typedef struct Station Relocation;

struct Param{
    double speed;
    bool HzrdLtIO;
    bool WindscenWipSt;
    bool Highbeem;
    bool Lowbeem;
    bool Windows;
    bool Doorlock;
    bool emergencybutton;
};

union Data{
    struct StationList *stationlist;
    struct NextStationList *nextstationlist;
    struct Station *relocation;
    struct Param param;
    bool start;
    bool stop;
    bool pause;
    bool resume;
};

struct Cloud{
    enum Cmd cmd;
    union Data data;
    QString *cmdId;
};

struct Point{
    double lon;
    double lat;
};

struct LaneProperties {
    std::vector<int> next_lane;
    double max_lat;
    double max_lon;
    double min_lat;
    double min_lon;
    int sum_point = 0;
};

struct StationProperties {
    int laneid;
    int pointid;
    Station station;
};

enum Station_Mode{
    nextstation = 1,
    stationlist  = 2,
    local = 0
};

#endif // STRUCT_H
