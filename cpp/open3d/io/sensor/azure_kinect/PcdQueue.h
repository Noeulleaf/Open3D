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
        std::lock_guard<std::mutex> lock(mtx);
        int size = _que.size();
        return size;
    }
	void push(std::shared_ptr<open3d::geometry::PointCloud> pcd);
    void pop();
    int size();
    std::shared_ptr<open3d::geometry::PointCloud> front();


private:
    std::mutex mtx;
    static CPcdQueue* _instance;
    std::queue<std::shared_ptr<open3d::geometry::PointCloud>> _que;
};
}  // namespace io
}  // namespace open3d
