//
// Created by sin on 2019/8/22.
// 通过对pipe进行封装,实现更好的父子进程之间的数据传输
//



#include <sstream>
#include "xpipe.h"

xpipe::xpipe() {
    int p = pipe(fd);
    if (p < 0) {
        perror("Couldn't create pipe, exit.");
        exit(-1);
    }
    readable = true;
    writeable = true;
}

xpipe::~xpipe() {
    if(this->readable){
        close(fd[0]);
    }
    if(this->writeable){
        close(fd[1]);
    }
}

void xpipe::set_write_only() {
    this->writeable = true;
    this->readable = false;
}

void xpipe::set_read_only() {
    this->readable = true;
    this->writeable = false;
}

ssize_t xpipe::send(void *buff, size_t length) {
    return write(fd[1], buff, length);
}

ssize_t xpipe::send(const string& data) {
    return write(fd[1], data.c_str(), data.length());
}

ssize_t xpipe::recv(void *buff, size_t length) {
    return read(fd[0], buff, length);
}

ssize_t xpipe::recv(string& data) {
    read(fd[0], buff, 4096);
    data = string(buff);
    return data.length();
}


void process_father::run() {
    for(int i=0;i<10;i++){
        stringstream d;
        d << i;
        p.send(d.str());
        printf("process father send %d\n", i);
        sleep(1.0);
    }
}

process_father::~process_father() {

}

void process_child::run() {
    for(int i=0;i<25;i++){
        string rec;
        p.recv(rec);
        printf("process child reveived %s\n", rec.c_str());
        sleep(0.5);
    }

}

process_child::~process_child() {

}

int main(int argc, char** argv){
    xpipe p;
    int pid = fork();

    if(pid < 0){
        perror("Error create fork, exit.");
        exit(-1);
    }
    if (pid == 0){
        p.set_read_only();
        process_child child(p);
        child.run();
    }else{
        p.set_write_only();
        process_father father(p);
        father.run();
    }
    return 0;
}