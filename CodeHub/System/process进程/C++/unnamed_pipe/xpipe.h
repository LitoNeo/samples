//
// Created by sin on 2019/8/22.
//

#ifndef TEST_XPIPE_H
#define TEST_XPIPE_H

#include <iostream>
#include <fstream>
#include <unistd.h>
using namespace std;

namespace{
    class xpipe{
    private:
        int fd[2]{};
        char buff[4096];
        bool readable;
        bool writeable;

    public:
        xpipe();
        ~xpipe();

        // 设置角色
        void set_write_only();
        void set_read_only();

        // 读写
        ssize_t send(void *buff, size_t length);
        ssize_t send(const string& data);

        ssize_t recv(void *buff, size_t length);
        ssize_t recv(string& data);
    };

    class process_father{
    private:
        xpipe p;
    public:
        process_father(const xpipe& p){
            this->p = p;
        };
        ~process_father();

        void run();
    };

    class process_child{
    private:
        xpipe p;
    public:
        process_child(const xpipe& p){
            this->p = p;
        };
        ~process_child();

        void run();
    };
}// end namespace

#endif //TEST_XPIPE_H
