#ifndef DECLERATIONS_H
#define DECLERATIONS_H


template<typename T>
class YcK {

public:
    YcK ():top1(-1), top2(MaxSize){}


    bool empty(int flag);
    bool push(T e, int flag);
    bool pop(int flag);
    bool top(T &e, int flag);

private:
    static const int MaxSize = 5;
    T data[MaxSize];
    int top1,top2;

};




#endif
