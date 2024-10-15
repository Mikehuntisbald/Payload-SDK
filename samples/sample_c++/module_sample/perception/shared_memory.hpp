#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <iostream>
#include <cstring>
#include <csignal>
#include <opencv2/opencv.hpp>

// #include <ros/ros.h>
// #include <sensor_msgs/Image.h>
// #include <cv_bridge/cv_bridge.h>

class SharedMemoryManager {
public:
    // 获取单例实例的静态方法
    static SharedMemoryManager& getInstance() {
        static SharedMemoryManager instance;
        return instance;
    }

    // 禁止拷贝构造和赋值操作
    SharedMemoryManager(const SharedMemoryManager&) = delete;
    SharedMemoryManager& operator=(const SharedMemoryManager&) = delete;

    // // 获取共享内存中的数据
    // std::string getSharedMemoryData() {
    //     try {
    //         if (region.get_address() == nullptr) {
    //             throw std::logic_error("Shared memory region is not mapped correctly.");
    //         }
    //         // 读取数据
    //         char* data = static_cast<char*>(region.get_address());
    //         return std::string(data);
    //     }
    //     catch (const boost::interprocess::interprocess_exception& ex) {
    //         std::cerr << "Error: " << ex.what() << std::endl;
    //         return "";
    //     }
    //     catch (const std::logic_error& ex) {
    //         std::cerr << "Logic error: " << ex.what() << std::endl;
    //         return "";
    //     }
    // }

    // // 写入数据到共享内存
    // void writeSharedMemoryData(const std::string& message) {
    //     try {
    //         if (region.get_address() == nullptr) {
    //             throw std::logic_error("Shared memory region is not mapped correctly.");
    //         }
    //         // 向共享内存中写入数据
    //         std::memcpy(region.get_address(), message.c_str(), message.size() + 1);
    //         std::cout << "Data written to shared memory: " << std::endl;
    //     }
    //     catch (const boost::interprocess::interprocess_exception& ex) {
    //         std::cerr << "Error: " << ex.what() << std::endl;
    //     }
    //     catch (const std::logic_error& ex) {
    //         std::cerr << "Logic error: " << ex.what() << std::endl;
    //     }
    // }

cv::Mat getSharedMemoryData(const int& position) {
    try {
        if (region.get_address() == nullptr) {
            throw std::logic_error("Shared memory region is not mapped correctly.");
        }

        // 从共享内存中读取图像元数据（行数、列数、类型）
        int* meta_data_ptr = static_cast<int*>(region.get_address());
        int rows = meta_data_ptr[0];
        int cols = meta_data_ptr[1];
        int type = meta_data_ptr[2];

        // 根据 position 读取不同的图像数据
        void* image_data_ptr = nullptr;
        if (position == 1) {
            std::cout << "read 1" << std::endl;
            image_data_ptr = reinterpret_cast<void*>(reinterpret_cast<char*>(meta_data_ptr) + 12);
        } else {
            std::cout << "read 2" << std::endl;
            image_data_ptr = reinterpret_cast<void*>(reinterpret_cast<char*>(meta_data_ptr) + 12 + rows * cols * CV_MAT_CN(type));
        }

        cv::Mat img(rows, cols, type, image_data_ptr);
        return img.clone(); // 返回图像数据的克隆，防止原数据被修改
    }
    catch (const boost::interprocess::interprocess_exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return cv::Mat();
    }
    catch (const std::logic_error& ex) {
        std::cerr << "Logic error: " << ex.what() << std::endl;
        return cv::Mat();
    }
}

void writeSharedMemoryData(const cv::Mat& img, const int& position) {
    try {
        if (region.get_address() == nullptr) {
            throw std::logic_error("Shared memory region is not mapped correctly.");
        }

        // 检查图像数据是否是连续的
        if (!img.isContinuous()) {
            throw std::logic_error("Input image data is not continuous.");
        }

        // 向共享内存中写入图像元数据（行数、列数、类型）
        int* meta_data_ptr = static_cast<int*>(region.get_address());
        meta_data_ptr[0] = img.rows;
        meta_data_ptr[1] = img.cols;
        meta_data_ptr[2] = img.type();

        // 根据 position 写入不同的图像数据
        void* image_data_ptr = nullptr;
        if (position == 1) {
            std::cout << "write 1" << std::endl;
            image_data_ptr = reinterpret_cast<void*>(reinterpret_cast<char*>(meta_data_ptr) + 12);
        } else {
            std::cout << "write 2" << std::endl;
            image_data_ptr = reinterpret_cast<void*>(reinterpret_cast<char*>(meta_data_ptr) + 12 + img.total() * img.elemSize());
        }

        std::memcpy(image_data_ptr, img.data, img.total() * img.elemSize());
        std::cout << "Image data written to shared memory." << std::endl;
    }
    catch (const boost::interprocess::interprocess_exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }
    catch (const std::logic_error& ex) {
        std::cerr << "Logic error: " << ex.what() << std::endl;
    }
}
    // 获取共享内存中的数据，返回为 cv::Mat 格式
    // cv::Mat getSharedMemoryData(const int& position) {
    //     try {
    //         if (region.get_address() == nullptr) {
    //             throw std::logic_error("Shared memory region is not mapped correctly.");
    //         }
    //         // 从共享内存中读取图像元数据（行数、列数、类型）
    //         int* meta_data_ptr = static_cast<int*>(region.get_address());
    //         int rows = meta_data_ptr[0];
    //         int cols = meta_data_ptr[1];
    //         int type = meta_data_ptr[2];

    //         // 从共享内存中读取图像数据
    //         if (position == 1){
    //             cout<<"read 1"<<endl;
    //             void* image_data_ptr = static_cast<void*>(meta_data_ptr + 3); // 图像数据紧随元数据之后
    //             cv::Mat img(rows, cols, type, image_data_ptr); // 使用共享内存指针创建 cv::Mat
    //             return img.clone(); // 返回图像数据的克隆，防止原数据被修改
    //         }else {
    //             cout<<"read 2"<<endl;
    //             void* image_data_ptr = static_cast<void*>(meta_data_ptr + 3 + rows * cols); // 图像数据紧随元数据之后
    //             cv::Mat img(rows, cols, type, image_data_ptr); // 使用共享内存指针创建 cv::Mat
    //             return img.clone(); // 返回图像数据的克隆，防止原数据被修改
    //         }

    //         // void* image_data_ptr = static_cast<void*>(meta_data_ptr + 3); // 图像数据紧随元数据之后
    //         // cv::Mat img(rows, cols, type, image_data_ptr); // 使用共享内存指针创建 cv::Mat
    //         // return img.clone(); // 返回图像数据的克隆，防止原数据被修改
    //     }
    //     catch (const boost::interprocess::interprocess_exception& ex) {
    //         std::cerr << "Error: " << ex.what() << std::endl;
    //         return cv::Mat();
    //     }
    //     catch (const std::logic_error& ex) {
    //         std::cerr << "Logic error: " << ex.what() << std::endl;
    //         return cv::Mat();
    //     }
    // }

    // // 写入 cv::Mat 图像数据到共享内存
    // void writeSharedMemoryData(const cv::Mat& img, const int& position) {
    //     try {
    //         if (region.get_address() == nullptr) {
    //             throw std::logic_error("Shared memory region is not mapped correctly.");
    //         }
    //         // 检查图像数据是否是连续的
    //         if (!img.isContinuous()) {
    //             throw std::logic_error("Input image data is not continuous.");
    //         }
    //         // 向共享内存中写入图像元数据（行数、列数、类型）
    //         int* meta_data_ptr = static_cast<int*>(region.get_address());
    //         meta_data_ptr[0] = img.rows;
    //         meta_data_ptr[1] = img.cols;
    //         meta_data_ptr[2] = img.type();

    //         // 向共享内存中写入图像数据
    //         if (position == 1){
    //             cout<<"write 1"<<endl;
    //             void* image_data_ptr = static_cast<void*>(meta_data_ptr + 3); // 图像数据紧随元数据之后
    //             std::memcpy(image_data_ptr, img.data, img.total() * img.elemSize()); // 复制图像数据到共享内存
    //         }else {
    //             cout<<"write 2"<<endl;
    //             void* image_data_ptr = static_cast<void*>(meta_data_ptr + 3 + img.total() * img.elemSize()); // 图像数据紧随元数据之后
    //             std::memcpy(image_data_ptr, img.data, img.total() * img.elemSize()); // 复制图像数据到共享内存
    //         }
    //         // void* image_data_ptr = static_cast<void*>(meta_data_ptr + 3); // 图像数据紧随元数据之后
    //         // std::memcpy(image_data_ptr, img.data, img.total() * img.elemSize()); // 复制图像数据到共享内存
    //         std::cout << "Image data written to shared memory." << img.total() * img.elemSize() << std::endl;
    //     }
    //     catch (const boost::interprocess::interprocess_exception& ex) {
    //         std::cerr << "Error: " << ex.what() << std::endl;
    //     }
    //     catch (const std::logic_error& ex) {
    //         std::cerr << "Logic error: " << ex.what() << std::endl;
    //     }
    // }

    void reSize(const int& rows, const int& cols, const int& type) {
        rows_ = rows;
        cols_ = cols;
        type_ = type;
        
        using namespace boost::interprocess;
        
        // 设置共享内存的大小
        shm.truncate(rows_ * cols_ * 2 + 12);

        // 重新映射共享内存到进程地址空间
        region = mapped_region(shm, read_write);
    }

    // 发布共享内存中的图像到 ROS
    // void publishImage() {
    //     try {
    //         cv::Mat img = getSharedMemoryData(); // 从共享内存获取图像数据
    //         if (!img.empty()) {
    //             sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
    //             image_pub.publish(msg); // 发布图像消息
    //             std::cout << "Image data published to ROS topic." << std::endl;
    //         } else {
    //             std::cerr << "No data available in shared memory to publish." << std::endl;
    //         }
    //     }
    //     catch (const std::exception& ex) {
    //         std::cerr << "Error during publishing: " << ex.what() << std::endl;
    //     }
    // }

private:
    // 私有化构造函数
    SharedMemoryManager() {
        try {
            using namespace boost::interprocess;
            boost::interprocess::shared_memory_object::remove("MySharedMemory");
            // 创建或打开共享内存对象
            shm = shared_memory_object(open_or_create, "MySharedMemory", read_write);

            // 设置共享内存的大小
            shm.truncate(sizeof(int) * 3 + 640 * 480);

            // 重新映射共享内存到进程地址空间
            region = mapped_region(shm, read_write);

            // 向共享内存中写入数据
            // char message[] = "Hello, Shared Memory!";
            // std::memcpy(region.get_address(), message, sizeof(message));
            // cv::Mat randomImage(1080, 1920, CV_8U);
            // cv::randu(randomImage, cv::Scalar::all(0), cv::Scalar::all(255));
            // writeSharedMemoryData(randomImage);
            // std::cout << "Data written to shared memory: " << std::endl;

            // 打开已存在的共享内存对象
            // shm = shared_memory_object(open_only, "MySharedMemory", read_only);

            // // 将共享内存映射到进程地址空间
            // region = mapped_region(shm, read_only);
            // 注册信号处理器以在程序结束时移除共享内存

            // 初始化 ROS 发布器
            // ros::NodeHandle nh;
            // image_pub = nh.advertise<sensor_msgs::Image>("/shared_memory_image", 1);

            std::signal(SIGINT, SharedMemoryManager::signalHandler);
            std::signal(SIGTERM, SharedMemoryManager::signalHandler);
        
        }
        catch (const boost::interprocess::interprocess_exception& ex) {
            std::cerr << "Error during initialization: " << ex.what() << std::endl;
        }
    }

    ~SharedMemoryManager() {
        removeSharedMemory();
    }

    static void signalHandler(int signal) {
        removeSharedMemory();
        std::exit(signal);
    }
    boost::interprocess::shared_memory_object shm;
    boost::interprocess::mapped_region region;
    // ros::Publisher image_pub;
    int rows_{640};
    int cols_{480};
    int type_{CV_8U};
        // 销毁共享内存
    static void removeSharedMemory() {
        boost::interprocess::shared_memory_object::remove("MySharedMemory");
    }
};

// int get() {
//     // 获取单例实例并调用接口读取共享内存数据
//     SharedMemoryManager& shmManager = SharedMemoryManager::getInstance();
//     std::string data = shmManager.getSharedMemoryData();
    
//     if (!data.empty()) {
//         std::cout << "Data read from shared memory: " << data << std::endl;
//     } else {
//         std::cout << "No data read from shared memory." << std::endl;
//     }

//     return 0;
// }