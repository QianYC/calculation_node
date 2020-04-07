//
// Created by yc_qian on 20-3-14.
//

#ifndef PI_ROBOT_PARAMETERREADER_HPP
#define PI_ROBOT_PARAMETERREADER_HPP

#include <iostream>
#include <map>

/**
 * robot state
 */
enum STATE {
    /**
     * 摆拍模式
     */
            STATIC = 0X0,
    /**
     * 摆拍，目标已确定
     */
            S_OBJECT_SELECTED = 0X1,
    /**
     * 摆拍，构图已确定
     */
            S_COMPOSITION_SELECTED = 0X2,
    /**
     * 摆拍，正在调整位置
     */
            S_ADJUSTING = 0X3,
    /**
     * 摆拍，位置已调整
     */
            S_POSITION_ADJUSTED = 0X4,
    /**
     * 抓拍模式
     */
            DYNAMIC = 0X8,
    /**
     * 抓拍，目标已确定
     */
            D_OBJECT_SELECTED = 0X10,

    /**
     * 没有找到目标，随机移动
     */
            ROAMING = 0X20
};

/**
 * robot motion
 */
enum MOTION {
    STOP = 0X0,
    FORWARD = 0X1,
    BACKWARD = 0X2,
    LEFT = 0X3,
    RIGHT = 0X4
};

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
