//
// Created by yc_qian on 20-3-14.
//

#ifndef PI_ROBOT_PARAMETERREADER_HPP
#define PI_ROBOT_PARAMETERREADER_HPP

#include <iostream>
#include <map>

class ParameterReader {
public:
    ParameterReader(std::string file = "../../../../params.txt"){
        std::ifstream fin(file);
        if (!fin) {
            std::cerr << "file not found : " << file << std::endl;
            return;
        }
        std::string line;
        while (!fin.eof()) {
            getline(fin, line);
            //ignore comment
            if (line[0] == '#') {
                continue;
            }
            int pos = line.find('=');
            if (pos == -1) {
                continue;
            }
            std::string key = line.substr(0, pos);
            std::string value = line.substr(pos + 1, line.length());
            data[key] = value;
        }
        fin.close();
    }

    std::string getString(std::string key) {
        return data[key];
    }

    double getDouble(std::string key) {
        return atof(getString(key).c_str());
    }

    int getInt(std::string key) {
        return atoi(getString(key).c_str());
    }
private:
    std::map<std::string, std::string> data;
};

#endif //PI_ROBOT_PARAMETERREADER_HPP
