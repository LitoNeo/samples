# 多线程(C++/Python)
本文包括一下内容：
　　通过C++11的标准库进行多线程编程，包括线程的创建/退出，线程管理,线程之间的通信和资源管理，以及最常见的互斥锁，另外对python下多线程的实现进行讨论。

[TOC]

# 前言
　　多线程模型共享同一进程资源，通过多线程可以极大的提高代码的效率，完成单一线程无法完成的任务。
几个需要记住的点：
C++中的线程是一个`类`，因此可以像操作类一样进行操作；
C++中的线程也是一类`资源`；

**sample**
```C++
#include <iostream>
#include <thread>

void Thread_1() { 
    std::cout << "This is Thread_1." << std::endl;
    return;
}

int main() {
    std::thread t{greeting};   // 列表初始化
    t.join();                 
    return 0;
}
```
以上是多线程下的`HelloWorld!`,从上我们可以看出C++多线程编程的基本步骤：
`创建线程函数 -> 实例一个线程 -> 运行`
**两个注意点**
1. 编译
我们使用了C++11的特性以及线程库pthread，因此在编译的时候这两个都要说明：
```shell
g++ --std=c++11 -pthread main.cpp
```
2. 线程初始化
> 从 C++ 11 开始，推荐使用**列表初始化{}**的方式，构造类类型的变量。

# 线程管理初步
包括`线程函数，启动线程，结束线程，线程传参`

## 1. 线程函数
任何事情都有个开始，`线程函数`就是新线程的开始入口。
线程函数必须是`callable`和无返回值的。
**普通函数**
例如上面例子中的简单形式`void func(void *params);`
**可调用类型的实例**
```c++
class ThreadTask {
 private:
    size_t count_ = 0;
 public:
    explicit ThreadTask (size_t count) : count_(count) {}
    void operator()() const {  // 定义callable
        do_something(this->count_);
    }
};

ThreadTask task{42};  // 初始化可调用类型的实例
std::thread wk_thread{task};  // 创建并初始化和运行新线程  // 列表初始化
```
*注意*:虽然callable的实例看起来和函数用法一样，但是其本质上仍然是一个类的对象，因此在传入线程进行初始化时，其会被`拷贝`到线程空间，因此callable的类在这里必须做好完善的拷贝控制(参[拷贝构造函数](https://zhuanlan.zhihu.com/p/77806109))

## 2. 线程启动
**线程随着thread类型实例的创建而创建**,因此线程就变成了如同实例一样的资源，由C++提供统一的接口进行管理。
**创建线程的三种不同的方式：**
1. 最简单最常见的方式
```c++
void thread_1();  // 创建线程函数

std::thread new_thread{thread_1};  // 通过列表初始化的方式，实例化一个线程
```
当函数的名字被拿来使用的时候，其实使用的是一个指针(隐式的)，当然我们也可以进行显式的使用`&thread_1`，二者表示的是一样的。

2. 通过可调用类型callable的实例创建
参见上方`线程函数：可调用类型的实例`。
注意，`强烈建议使用c++11的列表初始化方法`，尤其是使用临时构造的实例创建线程的时候:
```c++
std::thread new_thread1(CallableClass());  // 错误方式
std::thread new_thread2{CallableClass{}};  // 正确
```
3. 以`lambda-表达式`创建线程
[lambda表达式](https://www.jianshu.com/p/923d11151027)是c++中的可调用对象之一，在C++11中被引入到标准库中，使用时不需要包含任何头文件。

## 3. 线程结束
任何事情都有个结束。
　　当线程启动之后，我们必须在 std::thread 实例销毁之前，显式地说明我们希望如何处理实例对应线程的结束状态，尤其是线程内部调用了系统资源，比如打开串口和文件等等。未加说明，则会调用std::terminate()函数，终止整个程序。
**join()和detach()**
<img src="https://user-images.githubusercontent.com/44689665/62913122-820efe00-bdbd-11e9-9742-9b8cccb08137.png" />

如果选择接合子线程`t.join()`，则主线程会阻塞住，直到该子线程退出为止。
如果选择分离子线程`t.detach()`，则主线程丧失对子线程的控制权，其控制权转交给 C++ 运行时库。这就引出了两个需要注意的地方:
> 1. 主线程结束之后，子线程可能仍在运行（因而可以作为守护线程）；
> 2. 主线程结束伴随着资源销毁，需要保证子线程没有引用这些资源。

**异常退出/结束的处理**
　　以上所说的是正常结束退出的情况，但是在某些情况下线程会异常退出，导致整个程序终止。
　　*线程也是种一种资源*，因此我们可以考虑`RAII`的思想，构建一个ThreadGuard类来处理这种异常安全的问题。
> RAII: "资源获取即初始化",是C++语言的一种管理资源、避免泄漏的惯用法。其利用C++中的**构造的对象最终会被销毁的原则**,通过使用一个对象，在其构造时获取对应的资源，在对象生命期内控制对资源的访问，使之始终保持有效，最后在对象析构的时候，释放构造时获取的资源，因为析构函数一定会执行。

直接通过例子来说明：
```c++
struct ThreadGuard{
    private:
        std::thread& _t;
    public:
        explicit ThreadGuard(std::thread& t):_t(t){};
        ~ThreadGuard(){
            if (this->_t.joinable()){  // 如果线程没有结束，那么就等待线程结束
                this-_t.join();
            }
        }
        ThreadGuard(const ThreadGuard&) = delete;  // 禁止不必要的特殊成员函数
        ThreadGuard& operator=(const ThreadGuard&) = delete;
};

void func();

void do(){
    std::thread thread_1;
    ThreadGuard guard{thread_1};  // 传入ThreadGuard
    thread_1 = std::thread{func}; // 正常的线程创建和启动

    // .....
    return;
}

```
以上是一个典型的利用RAII保护资源的例子，无论do()进程如何退出，guard都会最终帮助thread_1确保退出。

## 4. 线程传参
> *共享数据的管理* 和 *线程间的通信* 是多线程编程的两大核心

**参数为引用类型时的处理**
> 注: 线程传递参数默认都是`值传递`, 即使参数的类型是引用,也会被转化
　　如果在线程中使用引用来更新对象时，就需要注意了。默认的是将对象拷贝到线程空间，其引用的是拷贝的线程空间的对象，而不是初始希望改变的对象.
解决方案:使用`std::ref()`
```c++
thread t(func, std::ref(data))
```
在创建和启动线程传入线程函数时，其需要采用`引用`方式的参数用`std::ref()`进行修饰，如此，在t线程中对data的修改会反馈到当前线程中。

**建议传参方式**
线程传参时，除了默认采用值传递，还会自动进行`格式转换`操作，这种操作有时是会出问题的，比如`const char*`强制转为`char`时。
因此，线程间进行传参建议采用结构体的方式，将参数统一包裹进来。
```c++
struct ThreadGuard{
    private:
        std::thread& _t;

    public:
        explicet ThreadGuard(std::thread& t):_t(t){};
        ~ThreadGuard(){
            if (this->_t.joinable()){
                this->_t.joinable();
            }
        }
        ThreadGuard(const std::thread&) = delete;
        ThreadGuard& operator=(const std::Thread&) = delete;
};

struct Param{  // 定义参数的结构体
    uint_8 thread_control;
    std::string name;
    ros::Publisher mode_publisher;
};

void thread_1(void *param){
    Param *_param = (Param *)param;
    std::string name = _param->name;
    // ...
}

void do(){
std::thread thread_1;
ThreadGuard guard{thread_1};
param = new Param();
// 构建param

thread_1 = std::thread{thread_1, param};

return;
}
```
以上为一个通过结构体进行传参，并使用RAII守护线程的完整例子。

**以类中非静态成员函数为线程函数**
前期在写USB2CAN驱动时，需要在同一个类中构建多个非静态成员函数并作为线程函数，特此记录。
```c++
class Task{
    public:
        void thread_1(int a);
        void do();
}

Task task;  // 1
std::thead{&Task::func, &task, 20};
```
该方法的使用注意事项:
> 1. 必须`显式`的使用函数的指针，并作为第一个参数 `&Task::fuc`
> 2. 其第一个参数必须是`类实例的指针`，且需要显式的传入  `&task`
> 3. 最后才是真正的参数
*因为是非静态函数，无法脱离实例单独存在，因此在使用之前必须保证相应的实例已经创建存在，且该实例的指针需要显式的传入线程创建函数中。*

## 5. 互斥锁
线程之间的锁有：`互斥锁、条件锁、自旋锁、读写锁、递归锁`。一般而言，锁的功能越强大，性能就会越低。
其中互斥锁使用的频率最高，本处也仅对互斥锁进行讨论。

**std::mutex**
　　`std::mutex`是C++11 中最基本的互斥量，`std::mutex`对象提供了独占所有权的特性——即不支持递归地对`std::mutex`对象上锁（而 `std::recursive_lock` 则可以递归地对互斥量对象上锁。）

*std::mutex的成员函数*
1. `lock()`: 调用线程将锁住该互斥量。线程调用该函数会发生下面 3 种情况：
    (1). 如果该互斥量当前没有被锁住，则调用线程将该互斥量锁住，直到调用 unlock之前，该线程一直拥有该锁。
    (2). 如果当前互斥量被其他线程锁住，则当前的调用线程被阻塞住。
    (3). 如果当前互斥量被当前调用线程锁住，则会产生死锁(deadlock)。
2. `unlock()`: 解锁，释放对互斥量的所有权。
3. `try_lock()`: 尝试锁住互斥量，如果互斥量被其他线程占有，则当前线程也不会被阻塞。线程调用该函数也会出现下面 3 种情况，
    (1). 如果当前互斥量没有被其他线程占有，则该线程锁住互斥量，直到该线程调用 unlock 释放互斥量。
    (2). 如果当前互斥量被其他线程锁住，则当前调用线程返回 false，而并不会被阻塞掉。
    (3). 如果当前互斥量被当前调用线程锁住，则会产生死锁(deadlock)。

**sample**
```c++
#include <thread>
#include <mutex>

volatile int counter(0);
std::mutex mutex;

void new_thread(){
    for(int i=0;i<100;i++){
        try(mutex.try_lock()){
            ++counter;
            mutex.unlock();
        }
    }
}

int main(int argc, char** argv){
    std::thread[10] threads[10];
    for(int i=0;i<10;i++){
        threads[i] = std::thread{new_thread};
    }

    for(auto& th:threads) th.join();

    return 0;
}
```

**std::lock_guard std::unique_lock**
在这个什么都讲究`智能`的时代，互斥所也不能跟不上潮流。  
`std::lock_guard std::unique_lock`与Mutex RAII相关，其智能性体现在如下两个方面：
1. 方便对互斥量上锁，不必手动解锁
2. RAII机制确保在崩溃或异常退出的情况下仍然能够正常释放锁

**sample**
二者在使用上是相似的，即在需要上锁的地方运行
```c++
#include <mutex> // std::mutex std::lock_guard std::unique_lock

std::mutex mutex;

// lock_guard
std::lock_guard<std::mutex> lck(mutex);

// unique_lock
std::unique_lock<std::mutex> lck(mutex);
```

# Python中的多线程
　　由于Python解释器的特性，Python对于cpu密集型的任务其加速效果并不明显。但是对于这一门“爬虫语言”，在大量的IO时用多线程还是很有必要的。

　　Python的标准库提供了两个模块：`_thread`和`threading`，`_thread`是低级模块，`threading`是高级模块，对`_thread`进行了封装。绝大多数情况下，我们只需要使用`threading`这个高级模块。

　　 与C++很相似，Python创建多线程也是创建一个线程实例，传入线程函数，不一样的地方在于Python需要手动调用`start()`以开始线程的执行，即创建和执行是分开的。

```python
import threading

def thread_new():
    print("This is thread {}".format(threading.current_thread().name))

t = threading.Thread(target=thread_new, args=(), name="HelloThread")
t.start()
t.join()  // join
```

## 1. 线程中的参数访问ThreadLocal
　　问题：如果有好几个线程都调用某个函数来进行数据处理，那么就得把数据每次都作为参数传入进去，每个函数都一层一层调用/传参，如下：
```python
def process_data_1(data):
    process_data_2(data)
    pass

def process_data_2(data):
    pass

def task_1(data):
    process_data_1(data)
    process_data_2(data)

def task_2(data):
    process_data_2(data)
    process_data_2(data)
```
可以看出以上参数的传递是非常复杂的。由于线程中的局部变量是只有当前线程能够访问的，因此这类参数的传递可以考虑使用线程中的“全局变量”来解决。
`ThreadLocal`就是解决这个问题的。
```python
import threading

local_school = threading.local()  # 创建在所有线程外的全局ThreadLocal对象

def process_data_1():
    data = local_school.student
    # process

def process_data_2():
    data = local_school.student
    # process

def task_1(data):
    local_school.student = data
    process_data_1()
    process_data_2()

# 以下正常启动线程
```
`local_school = threading.local()`相当于定义在全局中的一个`dict`，每个线程都可以访问得到，并修改/获取里面的数据，并且不同的线程进行的操作互不影响。
注意：
    1. threading.local()必须定义在所有线程之外
    2. 线程中必须先修改ThreadLock中的数据然后才能访问到

## 2. Python中的锁机制Theading.Lock()
Python对线程锁的实现也定义在Threading模块中，实现起来非常简单
```python
data = 0
lock = Threading.Lock()
def run_change():
    for i in range(100):
        lock.acquire()    # 获取锁
        try:
            data += 1
        finally:
            lock.release() # 释放锁
```