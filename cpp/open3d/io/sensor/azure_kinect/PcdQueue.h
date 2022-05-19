#pragma once

#include <queue>
#include <mutex>
#include "open3d/Open3D.h"
using namespace open3d::io;

namespace open3d {
namespace io {


class CPcdQueue {
private:
    CPcdQueue()=default;

public:
	static CPcdQueue* GetInstance(){
        if (_instance == nullptr) 
			_instance = new CPcdQueue();
        return _instance;
	}
        int Size() {
            _mutex.lock();
            int size = _que.size();
            _mutex.unlock();
            return size;
        }
	void push(std::shared_ptr<open3d::geometry::PointCloud> pcd);
    void pop();
    std::shared_ptr<open3d::geometry::PointCloud> front();


private:
    
    std::mutex _mutex;
    static CPcdQueue* _instance;
    std::queue<std::shared_ptr<open3d::geometry::PointCloud>> _que;
};
}  // namespace io
}  // namespace open3d
