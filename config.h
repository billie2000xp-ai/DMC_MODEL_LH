#ifdef SYSARCH_PLATFORM
#include "mininf/EmbededFile.h"
#include "HALib/Component.h"
#else
#include "assert.h"
using namespace std;
namespace LPDDRSim {
class Configurable {
    std::map<std::string, std::string> valmap;
    std::string nullstr;
    typedef uint64_t Uint64;
public:
    Configurable() : nullstr("") {
    }
    void getString(std::string buf) {
        std::string org(buf);
        if (org[0] == '#')
            return;
        std::string key = "", val = "";
        bool is_key = true;
        for (unsigned i = 0; i < org.size(); i++) {
            unsigned char ch = org[i];
            if (ch == '#') break;  // escape comment after #
            if (ch == '=') is_key = false;
            else if (org[i] != ' ' && org[i] != '\t') {
                if (is_key)
                    key += org[i];
                else
                    val += org[i];
            }
        }
        if (key.size() && val.size()) {
            valmap[key] = val;
        } else {
            std::cout<<"Error key: "<<key<<std::endl;
            assert(0);
        }
    }
    bool has(std::string key) {
        return valmap.find(key) != valmap.end();
    }
    std::string & get(std::string key) {
        auto it = valmap.find(key);
        if (it == valmap.end())
            return nullstr;
        else
            return it->second;
    }
    bool getBool(std::string key, bool _default = false) {
        auto it = valmap.find(key);
        if (it == valmap.end())
            return _default;
        std::string &val = it->second;
        if (val == "true" || val == "yes" || val == "on")
            return true;
        if (val == "false" || val == "no" || val == "off")
            return false;
        return _default;
    }
    Uint64 getUint64(std::string key, int base = 10) {
        auto it = valmap.find(key);
        istringstream iss(it->second);
        uint64_t value;
        if (it->second.find("0x") != string::npos) {
            if ((iss >> hex >> value).fail()) {
                std::cout<<"getUint64Hex Error key="<<key<<std::endl;;
            }
        } else {
            if ((iss >> dec >> value).fail()) {
                std::cout<<"getUint64Dec Error key="<<key<<std::endl;;
            }
        }
        return value;
    }
    unsigned getUint(std::string key, int base = 10) {
        auto it = valmap.find(key);
        istringstream iss(it->second);
        unsigned value;
        if (it->second.find("0x") != string::npos) {
            if ((iss >> hex >> value).fail()) {
                std::cout<<"getUint32Hex Error key="<<key<<std::endl;;
            }
        } else {
            if ((iss >> dec >> value).fail()) {
                std::cout<<"getUint32Dec Error key="<<key<<std::endl;;
            }
        }
        return value;
    }
    float getFloat(std::string key, int base = 10) {
        auto it = valmap.find(key);
        if (it == valmap.end()) {
            cout<<"getFloat Error key="<<key<<std::endl;;
            assert(0);
        }
        return std::stof(it->second);
    }
    std::vector<unsigned> getUintArray(std::string key, int base = 10) {
        auto it = valmap.find(key);
        if (it == valmap.end()) {
            std::cout<<"getUintArray Error key="<<key<<std::endl;;
            assert(0);
        }
        unsigned start = 0;
        unsigned end = 0;
        unsigned value = 0;
        std::vector <unsigned> ret;
        //MAP_CONFIG[key].clear();
        while (end < it->second.size()) {
            end = it->second.find(",", start + 1);
            istringstream iss(it->second.substr(start, end));
            iss >> dec >> value;
            ret.push_back(value);
            start = end + 1;
        }
        return ret;
    }
};

class SimpleConfig {
    std::map<std::string, std::string> valmap;
    std::string nullstr;
    typedef uint64_t Uint64;
public:
    SimpleConfig(const char *file) : nullstr("") {
        std::ifstream pfile(file);
        char buf[256];
        if (!pfile.good()) {
            std::cout << "Open file " << file << " failed.\n";
            return;
        }
        while (pfile.getline(buf, 256)) {
            std::string org(buf);
            if (org[0] == '#')
                continue;
            std::string key = "", val = "";
            bool is_key = true;
            for (unsigned i = 0; i < org.size(); i++) {
                unsigned char ch = org[i];
                if (ch == '#') break;  // escape comment after #
                if (ch == '=') is_key = false;
                else if (org[i] != ' ' && org[i] != '\t') {
                    if (is_key)
                        key += org[i];
                    else
                        val += org[i];
                }
            }
            if (key.size() && val.size())
                valmap[key] = val;
        }
    }
    bool has(std::string key) {
        return valmap.find(key) != valmap.end();
    }
    std::string & get(std::string key) {
        auto it = valmap.find(key);
        if (it == valmap.end())
            return nullstr;
        else
            return it->second;
    }
    bool getBool(std::string key, bool _default = false) {
        auto it = valmap.find(key);
        if (it == valmap.end())
            return _default;
        std::string &val = it->second;
        if (val == "true" || val == "yes" || val == "on")
            return true;
        if (val == "false" || val == "no" || val == "off")
            return false;
        return _default;
    }
    Uint64 getUint64(std::string key, int base = 10) {
        auto it = valmap.find(key);
        if (it == valmap.end()) {
            std::cout<<"getUint64 Error key="<<key<<std::endl;;
            assert(0);
        }
        return std::stoul(it->second, nullptr, base);
    }
    unsigned getUint(std::string key, int base = 10) {
        auto it = valmap.find(key);
        if (it == valmap.end()) {
            std::cout<<"getUint Error key="<<key<<std::endl;;
            assert(0);
        }
        return std::stoul(it->second, nullptr, base);
    }
    float getFloat(std::string key, int base = 10) {
        auto it = valmap.find(key);
        if (it == valmap.end()) {
            std::cout<<"getFloat Error key="<<key<<std::endl;;
            assert(0);
        }
        return std::stof(it->second);
    }
};
}
#endif