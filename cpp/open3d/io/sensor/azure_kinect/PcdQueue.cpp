// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018-2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------
#include "open3d/utility/Logging.h"
#include "PcdQueue.h"
#include <iostream>
#include <string>
using namespace open3d::io;

//#include "open3d/utility/Log\ging.h"

namespace open3d {
namespace io {

CPcdQueue* CPcdQueue::_instance = nullptr;

void CPcdQueue::push(std::shared_ptr<open3d::geometry::PointCloud> pcd) {
    constexpr int max_queue_size = 3;
    _mutex.lock();
    while(_que.size() >= max_queue_size) 
        _que.pop();    
    _que.push(pcd);
    _mutex.unlock();
}

void CPcdQueue::pop() { 
    _mutex.lock();
    _que.pop();
    _mutex.unlock();
}

std::shared_ptr<open3d::geometry::PointCloud> CPcdQueue::front() {
    _mutex.lock();
    std::shared_ptr<open3d::geometry::PointCloud> front_data = nullptr;
    if(_que.size()>0) {
        front_data = _que.front();
    }
    _mutex.unlock();
    return front_data;
}

}  // namespace io
}  // namespace open3d
