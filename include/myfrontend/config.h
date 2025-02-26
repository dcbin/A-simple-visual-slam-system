#ifndef CONFIG_H
#define CONFIG_H

#include "common_include.h"

namespace myfrontend {

// 配置类,使用单例模式,即程序中只允许存在一个Config实例
class Config {
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config() {} // 默认构造函数

public:
    ~Config() {} // 默认析构函数

    // 设置配置文件路径
    static void setParameterFile(const std::string& filename);

    // 获取配置文件中的数据
    template<typename T>
    static T getParam(const std::string& key) {
        return T(Config::config_->file_[key]);
    }
};

} // namespace myfrontend

#endif // CONFIG_H