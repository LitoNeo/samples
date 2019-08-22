# 多进程(C++/Python)
---
[TOC]
* 前言
* 多进程与多线程
* 多进程的实现
    1. 进程的结构
    2. 进程控制:fork() exec() waitpid()
* 多进程之间的通信
    1. 管道
    2. 消息队列
    3. 共享内存
    4. 信号量
    5. 套接字接口
* Python下的多进程

# 前言
　地方

# 多进程与多线程
　二者都是为任务的并发或并行执行而生的，一个任务可以由进程完成，也可以由线程完成，二者在完成任务上并没有本质上的区别。
　进程和线程最本质的区别，在于`隔离`与`共享`上的不同。

> 进程是操作系统进行`资源分配`和`执行`的最小单元。对于多进程来说，各个任务之间由操作系统保证了相互隔离。若要在多进程之间进行数据的传递、共享，必须要依赖操作系统信号、套接字、文件、管道等等。进程的切换和开销其开销较大。

> 线程是操作系统进行`调度`的最小单元。对于多线程来说，各个任务实际上处在同一进程空间，大多数的资源都能在内存空间中传递、共享，十分方便。但是另一方面，由于隔离不严，所以会出现十分棘手的「线程安全」问题。由于共享同一进程的资源，因此线程的切换和通信其开销较小。关于多线程详细内容可参见[多线程](https://zhuanlan.zhihu.com/p/77965207)

另外，目前C++标准仅支持了多线程，尚没有支持多进程，C++下进行多进行变成还要用到库函数和系统调用。

*以下关于C++中关于多进程的内容均建立在Linux系统之上。*

# 多进程的实现
## 1. 进程的结构
　一个进程的结构可分为:`代码段`，`堆/栈段`，`数据段`
* `代码段`：存放程序代码，是固定的。因此如果有多个进行运行相同的一个程序，那么其代码段是同一个；
* `堆/栈段`：存放`子程序的返回地址，子程序的参数，程序的局部变量`；
* `数据段`：存放程序的全局变量，常数，动态分配的空间。

关于三个结构的细节不在此展开，只要知道有这样三段结构体及其负责的任务不同即可。

## 2. 进程控制
### （1）fork() 拷贝进程
<!-- <img src="fork" /> -->
`fork`意指分叉，指`拷贝`当前进程并并行运行,如上所示。
> 关于`拷贝`的理解：默认情况下fork出来的子进程会将当前进程完全拷贝一份，包括代码段，堆栈段，数据段，但是也可以对其中的数据有选择的拷贝。拷贝后，父子两进程就成为并行的进程了，其数据段(程序代码)是完全一样的(同一份)，堆栈段和数据段则是相互独立，修改互不影响。

> 新建立的子进程并`不会`继承父进程的线程，即新建的子进程只有一个线程，对应父进程中调用fork()的那个线程。由于主进程的整个虚存空间都被复制到了子进程，包括`互斥锁，条件变量，其余线程的内部对象等`，因此如果主进程中其他线程修改了这些对象，并在之后调用了fork(),那么这些操作结果会被继承到子进程中，造成安全问题。(比如父进程定义一把互斥锁，且在调用fork()之前这把锁被锁住了，那么fork()之后的子进程中，其这把锁的初始状态就是锁住的，且无法解锁--因为两个进程是独立的，父进程中的锁打开并不会影响到子进程中的锁的状态)。--当然有一些方法可以帮助解决这类问题，但是这类问题目前`没有银弹`，即没有一个完全可靠的方法能保证解决好这类问题，因此**在多线程下执行fork()是危险的，尽量不要使用多进程和多线程混合的模型**

`fork()`作为系统调用之一，其独立于C++进程而存在。调用一次fork()会有两次返回。
> 其实这里并不难理解。fork()引起分叉，创立了子进程，如上图所示，从图中也可以看出，子进程就是从fork()处开始执行的。父子进程的代码段是完全一样的，父进程里的fork()返回子进程的pid，子进程里的fork()返回0，指代自身。程序代码通过fork()的返回值可以确认当前执行的到底是哪个进程，用以执行不同的代码段或者进行进程控制。

辅以示例代码进行说明:
**sample**
```c++
#include <sys/types.h> /* 提供类型pid_t的定义 */
#include <unistd.h>  // unix standard库的缩写，包含一系列的库函数
#include <stdio.h>

void func_1();
void func_2();

int main(int argc, char** argv){
    const int cnt = 0;
    
    pid_t pid = fork();  // 从此处开始分叉，创建了子进程，且子进程从这一行开始运行
    if (pid < 0 ){  // 有效的pid都是非负整数
        perror("Fork failed");
    } else if (pid == 0){ // 子进程，fork()返回0
        printf("I am child.");
        func_2();
    }else{  // 父进程，fork()返回子进程的pid
        printf("I am father.")
        func_1();  
    }

    return 0;
}

```
另外值得一说的是，如果父进程kill掉了，其子进程会继续运行，且会被挂到`init`系统进程下，`init`进程变成其父进程了。

### （2）exec() 新建进程
　`exec()`不是单一库函数，而是一系列库函数，包括`execl(), execlp(),execle()等等`，但是这些函数最终都会调用
`int execve(const char *filename, char *const argv[], char *const envp[]);`
其参数为(可执行文件名称，参数列表，NULL) --**最后一个必须为NULL**
例如
```c++
char *argv[] = {"/home","-l",NULL};
ret = execvp("ls",argv);
```

　该函数会`反客为主`，把当前进程整个清空，并装在filename指代的可执行文件(二进制或脚本都可，只要可执行)。
> 和`fork()`一样，该函数一经调用立即执行，因此**execve()之后的原来的代码就都不会执行了**，因为都被洗掉了，这一点要注意。

**sample**
/home/neo/shell/test.sh
```shell
#!/usr/bin/env bash

echo "I am shell"
exit 0
```

```c++
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>

int main(int argc, char** argv){
    const int cnt = 0;

    printf("I am ready to create new process");

    if(execl("/home/neo/shell/test.sh", NULL) == -1){  // 失败则返回-1
        perror("execl failed");
    }else{
        printf("Creat new process successful");  // 当且仅当execl()执行失败时，main才会继续往下进行
    }
    return 0
}
```
### （3）fork()+exec()创建并行新进程
`fork()`可以创建一个并行运行的拷贝后进程；
`exec()`可以开启一个新的程序替代当前进程；
--那么想当然的，`fork()+exec()`可以组合起来创建并行运行的新进程，即在子进程里调用exec()创建新进程即可，此处不做详述。

### （4）fork() + wait()/waitpid()进程控制
　`fork()`创建的父子进程间天然存在`信号`的通信，当子进程终止(正常/异常)的时候，内核就会向其父进程发送SIGCHLD信号。父进程接收到信号后可以有选择的进行处理。
　如果某个进程结束但是其父进程还没有调用wait(),那么该进程就会成为`僵尸进程`--仅在进程列表中保留一个位置，记录其退出状态信息供查询，没有代码段等了，几乎不占内存。从这个角度看，`wait()/waitpid()`倒像是一个僵尸收割器。

函数原型
```c++
#include <sys/wait.h>

pid_t wait(int *status);

pid_t waitpid(pid_t pid, int *status, int options);
```
**wait()**
`wait()`会监听当前所有子进程的状态，*一旦调用，则当前进程被阻塞*，知道被僵尸进程激活。因此`wait()`是一个`循环的僵尸进程收割器`。 --**注意，如果当前进程没有子进程，则wait()会直接返回失败(-1)而退出**

`pid_t wait(int *status);`
　从其原型定义也可以看出，`wait()`会返回僵尸进程的`pid号`，并将其状态保存到`*status`中以供查询(NULL则表示不在乎其状态而直接销毁进程)。
　对`*status`的解析由专门的宏来进行:
1. `WIFEXITED(status)`解析是否正常退出，返回0表示正常(`return 0`)，否则异常;
2. `WEXITSTATUS(status)`用来对解析异常退出的值，比如进程`exit(5)`,则该宏将解析出5。

**wait() sample**
```c++
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdlib.h>

int main(int argc,char** argv){
    pit_t p1,p2;
    p1 = fork();
    if (p1 < 0){
        perror("fork failed");
        exit(1);
    }else if(p1 == 0){  // 子进程
        printf("child sleep for 10s");
        sleep(10);
    }else{  // 父进程
        p2 = wait(NULL);  // 父进程在此阻塞，用p2接收结束的进程号，不保存其结束状态
        printf("child %ld ended" % long(p2));
    }
    return 0;
}
```

**waitpid()**
`waitpid()`作用和`wait()`一样，不一样的地方在于：
1. `waitpid()`可以监听制定进程结束状态，通过第一个参数pid指定；
2. `waitpid()`提供`int options`参数进行额外控制：
    （1）`WNOHANG`:使用该参数，则当前进程不会阻塞在此--若无僵尸进程则返回0；即该函数只会监听一次，以后要监听还要重新调用。
     (2) `WUNTRACED`跟踪调试相关，此处暂且不考虑。

**waitpid() sample**
```c++
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdlib.h>

int main(int argc,char** argv){
    pit_t p1,p2;
    p1 = fork();
    if (p1 < 0){
        perror("fork failed");
        exit(1);
    }else if(p1 == 0){  // 子进程
        printf("child sleep for 10s");
        sleep(10);
    }else{  // 父进程
        p2 = wait(p1, NULL, WNOHANG);  // 父进程:监听一次p1，不记录结束状态，不阻塞
        if (p2 == p1){
            printf("child %ld ended" % long(p2));
        }else{
            sleep(1);
        }
    }
    return 0;
}
```

# 多进程之间的通信

## 1. 管道
Linux系统中，管道pipe也是一种文件，大小为4096字节，实际是位于内存或文件系统中的中的一段`RingBuffer`。
管道是Unix中最古老的一种进程通信方式了，其只能用于同一台电脑中进程之间的通信，且速度较慢。
原型：

### 无名管道
无名管道**只能**用于父子进程之间的通信，为`半双工方式`。
```c++
#include <unistd.h>

int fd[2];
int pipe(fd);  // 成功返回0，失败返回-1
```
当一个管道建立时，会同时创建两个文件描述符，**fd[0]为读而打开，fd[1]为写而打开**。
管道两端根据自己需求选择读还是写，并关闭另一个文件描述符。
要彻底关闭管道，只需要将这两个文件描述符都关闭即可。

**sample**
在`fork()之前`调用`pipe()`，以使得子进程能够获得这两个文件描述符。
```c++
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <sys/types.h>

int main(int argc, char** argv){
    int fd[2];
    pid_t pid;
    char data[4096];  // 子进程接收数据的容器
    int p = pipe(fd);

    if(p == -1){
        perror("Create pipe error");
        exit(-1);
    }

    pid = fork();
    if(pid < 0){
        perror("Create child process error");
        exit(-1);
    }else if(pid == 0){ // 子进程,读
        close(fd[1]);   // 关闭写端
        read(fd[0],data,4096);
        printf("child process received %s\n",data);
    }else{              // 父进程,写
        close(fd[0]);   // 关闭读端
        std::string sig = "father sig";
        write(fd[1], sig.c_str(), sig.size());
        printf("father process send %s\n", sig.c_str());
    }
    return 0;
}
```
注:在使用时,一般需要对pipe进行一定的封装,具体可以参考[xpipe代码](https://github.com/LitoNeo/samples/blob/master/CodeHub/System/process%E8%BF%9B%E7%A8%8B/C%2B%2B/unnamed_pipe/xpipe.cpp)

### 命名管道FIFO
**与无名管道的不同之处**
> 1. 命名管道是一种特殊的文件类型(特殊设备文件)，存在于文件系统中，有`文件名`与之相对应
> 2. 命名管道用于无关的进程之间进行数据的交换

```c++
#include <sys/stat.h>

int mkfifo(const char *path, mode_t mode);  // make FIFO, 成功返回0，失败返回-1；
```
由于命名管道是一种文件，因此其操作也是和操作文件的方式非常像的，一旦创建了一个FIFO，就可以一般的文件io方式进行操作；
`path`表示文件的文件名，通常指定一个string即可；
`mode`指定文件的读写权限，比如`0666`;

**注意，mkfifo函数只是创建一个FIFO文件，要使用命名管道还是通过文件操作(`open`)将其打开。**

**关于函数open**
C++中的`open`函数是个经常用到的文件操作函数，但是仍有几个地方需要注意一下:
```c++
open(const char* path, O_RDONLY);               // 只读，阻塞(默认)
open(const char* path, O_RDONLY | O_NONBLOCK);  // 只读，非阻塞
open(const cahr* path, O_WRONLY);               // 只写，阻塞
open(const char* path, O_WRONLY | O_NONBLOCK);  // 只写，非阻塞
```
> 阻塞,(默认):`只读open`要阻塞到某个其他进程为写而打开此FIFO，`只写open`要阻塞到某个其他进程为读而打开它 --相当于两个人打电话,必须两个人都拿起话筒后才能通信,否则等待;
> `O_NONBLOCK`非阻塞:`只读open`立即返回,`只写open`出错返回-1(当没有读端时);

**[sample]()**
此处我们使用C++的`fstream`进行实现.
有名管道的实现分为以下4步:
> 1. mkfifo()创建管道
> 2. open()打开管道
> 3. 进行管道的读写
> 4. 关闭文件/管道

writer.cpp
```c++
#include <unistd.h>
#include <fstream>
#include <sys/stat.h>  // mkfifo

int main(int argc, char** argv){
    if((mkfifo("fifo_1", 0666)) == -1){
        printf("file exists\n");
    };

    std::fstream file;
    file.open("fifo_1",std::ios::out);
    for(int i=0;i<5;i++){
        file << i << std::endl;
        printf("writer send message %d\n", i);
        sleep(1);
    }
    file.close()
    return 0;
}
```

reader.cpp
```c++
#include <fstream>
#include <sys/stat.h>  // mkfifo
#include <iostream>

int main(int argc, char** argv){
    if((mkfifo("fifo_1", 0666)) == -1){
        printf("file exists\n");
    };

    std::fstream file;
    file.open("fifo_1",std::ios::in);
    for(int i=0;i<5;i++){
        std::string data;
        file >> data;
        std::cout << "reader receive message " << data << std::endl;
    }
    file.close();
    return 0;
}
```

http://songlee24.github.io/2015/04/21/linux-IPC/

https://blog.csdn.net/kobejayandy/article/details/18863543

https://www.jianshu.com/p/9218692cb209

https://www.cnblogs.com/dk666/p/7412527.html

* 多进程之间的通信
    1. 管道
    2. 消息队列
    3. 共享内存
    4. 信号量
    5. 套接字接口
* Python下的多进程