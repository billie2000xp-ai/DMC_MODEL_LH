#ifndef INIREADER_H
#define INIREADER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include "SystemConfiguration.h"
#include "ddr_common.h"
#include "config.h"

using namespace std;

#define DEFINE_UINT_PARAM(name, paramtype) {#name, &name, UINT, paramtype, false}
#define DEFINE_STRING_PARAM(name, paramtype) {#name, &name, STRING, paramtype, false}
#define DEFINE_FLOAT_PARAM(name,paramtype) {#name, &name, FLOAT, paramtype, false}
#define DEFINE_BOOL_PARAM(name, paramtype) {#name, &name, BOOL, paramtype, false}
#define DEFINE_UINT64_PARAM(name, paramtype) {#name, &name, UINT64, paramtype, false}
#define DEFINE_UINT64HEX_PARAM(name, paramtype) {#name, &name, UINT64HEX, paramtype, false}
#define DEFINE_UINTARRAY_PARAM(name, paramtype) {#name, &name, UINTARRAY, paramtype, false}
#define DEFINE_FLOATARRAY_PARAM(name, paramtype) {#name, &name, FLOATARRAY, paramtype, false}

namespace LPDDRSim {

typedef enum _variableType {STRING, UINT, UINT64, FLOAT, BOOL, UINT64HEX, UINTARRAY, FLOATARRAY} varType;
typedef enum _paramType {SYS_PARAM, DEV_PARAM} paramType;
typedef struct _configMap {
    string iniKey; //for example "tRCD"

    void *variablePtr;
    varType variableType;
    paramType parameterType;
    bool wasSet;
} ConfigMap;

class IniReader {

public:
    typedef std::map<string, string> OverrideMap;
    typedef OverrideMap::const_iterator OverrideIterator;

    static void SetKey(string key, string value, bool isSystemParam = false, size_t lineNumber = 0);
    static void OverrideKeys(const OverrideMap *map);
    static void ReadIniFile(istream &iniFile, bool isSystemParam);
#ifdef SYSARCH_PLATFORM
    static void ModifyParameter(HALib::Configurable* cfg);
#else
    static void ModifyParameter(Configurable cfg);
#endif
    static void CheckParameter();
    static void InitEnumsFromStrings();
    static bool CheckIfAllSet();
    static void WriteValuesOut(std::ofstream &visDataOut);
    static int getBool(const std::string &field, bool *val);
    static int getUint(const std::string &field, unsigned int *val);
    static int getUint64(const std::string &field, uint64_t *val);
    static int getUint64Hex(const std::string &field, uint64_t *val);
    static int getFloat(const std::string &field, float *val);

private:
    static void WriteParams(std::ofstream &visDataOut, paramType t);
    static void Trim(string &str);
};

enum b_type {
    START,
    END,
    DAT
};

struct line_message {
    std::string operation;
    uint32_t address_l;
    uint32_t address_h;
    uint32_t len;
    uint32_t id;
    b_type issue;
    uint64_t task;

    line_message() {
        operation = "Read";
        address_h = 0;
        address_l = 0;
        len = 0;
        id = 0;
        issue = START;
        task = 0;
    }
};

class getfile {

    line_message nullstr;
public:
    std::map<unsigned long,line_message> start_valmap;
    std::map<unsigned long, line_message> end_valmap;
    std::map<unsigned long, line_message> data_valmap;
    getfile(string filename);

    getfile() {
    }

    line_message & get(unsigned long key, b_type type) {
        if (type == START) {
            auto it = start_valmap.find(key);
            if (it == start_valmap.end())
                return nullstr;
            else
                return it->second;
        } else if (type == END) {
            auto it = end_valmap.find(key);
            if (it == end_valmap.end())
                return nullstr;
            else
                return it->second;
        } else if (type == DAT) {
            auto it = data_valmap.find(key);
            if (it == data_valmap.end())
                return nullstr;
            else
                return it->second;
        }

    }

    bool has(unsigned long key,b_type type) {
        if (type == START) {
            return (start_valmap.find(key) != start_valmap.end());
        } else if (type == END) {
            return (end_valmap.find(key) != end_valmap.end());
        } else if (type == DAT) {
            return (data_valmap.find(key) != data_valmap.end());
        }
    }

    friend ostream &operator<<(ostream &os ,const line_message &t);
};
}
#endif